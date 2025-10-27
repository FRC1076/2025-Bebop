# Part 7: Robot Lifecycle

Understanding how a robot program starts, runs, and transitions between modes is crucial for debugging and development. Let's trace the entire lifecycle from power-on to match play.

## The Entry Point: Main.java

**File:** [Main.java](src/main/java/frc/robot/Main.java)

```java
public final class Main {
    private Main() {}

    public static void main(String[] args) {
        RobotBase.startRobot(Robot::new);
    }
}
```

**What happens:**
1. Java starts here (like `main()` in any Java program)
2. `RobotBase.startRobot()` is called with a reference to create a `Robot`
3. WPILib's framework takes over

**You never need to modify this file!**

## The Robot Class

**File:** [Robot.java](src/main/java/frc/robot/Robot.java)

This class extends `LoggedRobot` (from AdvantageKit), which extends `TimedRobot` (from WPILib).

### Constructor - Robot Power-On

**File:** [Robot.java:38-64](src/main/java/frc/robot/Robot.java#L38-L64)

```java
public Robot() {
    Logger.recordMetadata("ProjectName", "2025-Bebop");

    if (SystemConstants.currentMode == RobotMode.REAL) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to USB
        Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables
    } else if (SystemConstants.currentMode == RobotMode.SIM) {
        Logger.addDataReceiver(new NT4Publisher());
    } else {
        // Replay mode
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    Logger.start();

    m_robotContainer = new RobotContainer();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    RobotContainer.threadCommand().schedule();
}
```

**Step-by-step:**

1. **Configure logging** based on mode:
   - **REAL**: Log to USB stick + NetworkTables
   - **SIM**: NetworkTables only (no physical USB)
   - **REPLAY**: Read from log file, replay as fast as possible

2. **Start logging**: `Logger.start()` - after this, no more config allowed!

3. **Create RobotContainer**: This is where EVERYTHING gets created
   - All subsystems
   - All commands
   - All button bindings

4. **Get auto command**: Store for later use

5. **Schedule thread priority command**: After 20 seconds, raises priority for better performance

**This runs ONCE when robot powers on!**

## RobotContainer - The Factory

**File:** [RobotContainer.java:77-130](src/main/java/frc/robot/RobotContainer.java#L77-L130)

```java
public RobotContainer() {
    m_beamBreak = new BeamBreak(OIConstants.kBeamBreakPin);

    if (SystemConstants.currentMode == RobotMode.REAL) {
        m_arm = new ArmSubsystem(new ArmIOHardware());
        m_index = new IndexSubsystem(new IndexIOHardware());
        m_intake = new IntakeSubsystem(new IntakeIOHardware());
        m_shooter = new ShooterSubsystem(new ShooterIOHardware());

        m_drive = new DriveSubsystem(
            new GyroIOPigeon(),
            new ModuleIOHardware(ModuleConfig.FrontLeft),
            new ModuleIOHardware(ModuleConfig.FrontRight),
            new ModuleIOHardware(ModuleConfig.RearLeft),
            new ModuleIOHardware(ModuleConfig.RearRight)
        );
    } else {
        m_arm = new ArmSubsystem(new ArmIODisabled());
        m_index = new IndexSubsystem(new IndexIODisabled());
        m_intake = new IntakeSubsystem(new IntakeIODisabled());
        m_shooter = new ShooterSubsystem(new ShooterIODisabled());

        m_drive = new DriveSubsystem(/* ... */);
    }

    m_superstructure = new Superstructure(
        m_arm,
        m_index,
        m_intake,
        m_shooter,
        m_beamBreak.beamBrokenSupplier()
    );

    m_drive.setDefaultCommand(/* ... */);

    configureDriverBindings();
}
```

**Creation order matters!**
1. Sensors (beam break)
2. Subsystems (with appropriate IO implementations)
3. Superstructure (coordinates subsystems)
4. Default commands
5. Button bindings

**This runs ONCE during Robot constructor!**

## Robot Modes

The robot can be in one of several modes:

### Disabled Mode
Robot is powered but not moving.

**File:** [Robot.java:83-87](src/main/java/frc/robot/Robot.java#L83-L87)

```java
@Override
public void disabledInit() {}

@Override
public void disabledPeriodic() {}
```

**When:**
- Right after boot
- Between matches
- When E-stopped
- When drivers disable

**What runs:**
- `robotPeriodic()` (scheduler runs)
- `disabledPeriodic()`
- All subsystem `periodic()` methods
- Logging continues

**What doesn't run:**
- Commands (scheduler cancels them)
- Motor outputs (disabled by hardware)

### Autonomous Mode
Robot runs pre-programmed routines (first 15 seconds of match).

**File:** [Robot.java:90-102](src/main/java/frc/robot/Robot.java#L90-L102)

```java
@Override
public void autonomousInit() {
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
}

@Override
public void autonomousPeriodic() {}
```

**When:**
- First 15 seconds of match
- Or when enabled in auto mode on driver station

**What happens:**
1. `autonomousInit()` runs once
2. Schedules the auto command
3. `autonomousPeriodic()` runs every 20ms (but usually empty)
4. `robotPeriodic()` still runs (scheduler!)

### Teleop Mode
Drivers control the robot (rest of match after auto).

**File:** [Robot.java:105-117](src/main/java/frc/robot/Robot.java#L105-L117)

```java
@Override
public void teleopInit() {
    if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
    }
}

@Override
public void teleopPeriodic() {}
```

**When:**
- After autonomous ends
- Last ~2 minutes of match

**What happens:**
1. `teleopInit()` runs once
2. Cancels auto command (so drivers can take control)
3. Default commands start (like drive teleop)
4. Button bindings become active
5. `robotPeriodic()` still runs!

### Test Mode
For testing individual subsystems (rarely used in competition).

**File:** [Robot.java:120-127](src/main/java/frc/robot/Robot.java#L120-L127)

```java
@Override
public void testInit() {
    CommandScheduler.getInstance().cancelAll();
}

@Override
public void testPeriodic() {}
```

**When:** Selected on driver station (not during matches)

## The Periodic Heartbeat

The most important method in the robot:

**File:** [Robot.java:74-80](src/main/java/frc/robot/Robot.java#L74-L80)

```java
@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();
}
```

**This runs EVERY 20ms, in ALL modes!**

### What CommandScheduler.run() Does

Every 20ms (50 Hz):

```
CommandScheduler.run():
  1. Poll all triggers (buttons, sensors, etc.)
  2. Check if any new commands should start
  3. Check subsystem requirements
     - If new command needs a subsystem, cancel old command using it
  4. For each running command:
     a. Call execute()
     b. Check isFinished()
     c. If finished, call end(false)
     d. Remove from running list
  5. For each subsystem:
     a. Call periodic()
  6. Update logging data
```

**This is the engine of the entire robot!**

### Timing Guarantee

WPILib guarantees:
- `robotPeriodic()` called every 20ms ± a few microseconds
- Very consistent timing
- Even if code takes longer, next cycle starts on time (may skip logging)

## The Call Stack

When robot is running in teleop:

```
Every 20ms:
  main()
    └─ RobotBase.startRobot()
         └─ Robot.robotPeriodic()
              └─ CommandScheduler.getInstance().run()
                   ├─ Poll triggers
                   │    └─ m_driverController.leftTrigger().whileTrue(...)
                   │
                   ├─ Run commands
                   │    ├─ DriveClosedLoopTeleop.execute()
                   │    │    └─ m_drive.driveCLCO()
                   │    │
                   │    └─ IntakeCommand.execute()
                   │         └─ m_intake.setVoltage()
                   │              └─ io.setVoltage()
                   │
                   └─ Run subsystem periodic()
                        ├─ m_drive.periodic()
                        │    ├─ io.updateInputs(inputs)
                        │    └─ Logger.processInputs("Drive", inputs)
                        │
                        ├─ m_arm.periodic()
                        │    ├─ io.updateInputs(inputs)
                        │    ├─ Run PID if enabled
                        │    └─ Logger.processInputs("Arm", inputs)
                        │
                        └─ (all other subsystems...)
```

## AdvantageKit Logging

The logging happens in two places:

### 1. Subsystem Periodic
**File:** [ArmSubsystem.java:84-96](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java#L84-L96)

```java
@Override
public void periodic() {
    io.updateInputs(inputs);  // Read from hardware

    // ... control logic ...

    Logger.processInputs("Arm", inputs);  // Log the data
}
```

**Every subsystem does this!**

### 2. Automatic Outputs

**File:** [Superstructure.java:56-57](src/main/java/frc/robot/subsystems/Superstructure.java#L56-L57)

```java
@AutoLogOutput(key="SuperState/MechanismState")
public String getMechanismStateString() {
    return this.mechanismState.toString();
}
```

`@AutoLogOutput` automatically logs the return value every cycle!

## Thread Priority Command

**File:** [RobotContainer.java:221-227](src/main/java/frc/robot/RobotContainer.java#L221-L227)

```java
public static Command threadCommand() {
    return Commands.sequence(
        Commands.waitSeconds(20),
        Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 1)),
        Commands.print("Main Thread Priority raised to RT1 at " + Timer.getFPGATimestamp())
    ).ignoringDisable(true);
}
```

**Why?**
- After 20 seconds, raises main thread to real-time priority
- Improves timing consistency
- Only after robot has stabilized (not during boot glitches)

**`.ignoringDisable(true)`:**
- Command runs even in disabled mode
- Ensures priority gets set regardless of mode

## Common Initialization Patterns

### Pattern 1: Lazy Initialization
Don't create objects until needed:

```java
private Subsystem subsystem;

public Subsystem getSubsystem() {
    if (subsystem == null) {
        subsystem = new Subsystem();
    }
    return subsystem;
}
```

**(Not used in this codebase - we create everything in constructor)**

### Pattern 2: Mode-Based IO Selection
```java
if (SystemConstants.currentMode == RobotMode.REAL) {
    subsystem = new Subsystem(new HardwareIO());
} else {
    subsystem = new Subsystem(new DisabledIO());
}
```

**(Used throughout this codebase!)**

### Pattern 3: Separated Configuration
```java
public RobotContainer() {
    createSubsystems();
    setDefaultCommands();
    configureBindings();
}
```

**(Partially used - bindings are in separate method)**

## Try It Yourself

### Exercise 1: Trace Robot Boot
Starting from power-on, list the order of execution:
1. What file's `main()` runs first?
2. What class is instantiated next?
3. What happens in the Robot constructor?
4. What happens in RobotContainer constructor?
5. When does the first `periodic()` run?

### Exercise 2: Mode Transitions
If robot transitions from Disabled → Auto → Teleop, what methods run?

```
Robot boots:
  → ??

Driver enables Auto:
  → ??

Auto ends, Teleop starts:
  → ??
```

### Exercise 3: Understand Periodic Timing
If a command's `execute()` takes 15ms to run, and you have 5 commands running:
1. How long does one cycle take?
2. Does this affect the 20ms timing?
3. What happens if it takes 25ms total?

### Exercise 4: Logging Flow
Trace how arm position gets logged:
1. Where is the sensor read?
2. Where is it stored?
3. When does `Logger.processInputs()` get called?
4. Where does the data go?

### Exercise 5: Find Init Methods
Search through the code:
1. How many `@Override public void periodic()` methods are there?
2. Are there any `initialize()` methods in commands?
3. Are there any `end()` methods?

## Lifecycle Comparison

| Method | Frequency | Purpose | Commands Run? |
|--------|-----------|---------|---------------|
| `Robot()` | Once | Setup logging, create container | No |
| `RobotContainer()` | Once | Create subsystems, bind buttons | No |
| `robotPeriodic()` | Every 20ms | Run scheduler | Yes |
| `disabledInit()` | Once per disabled | Setup for disabled | Canceled |
| `disabledPeriodic()` | Every 20ms in disabled | Usually empty | No |
| `autonomousInit()` | Once per auto | Schedule auto command | Yes (starts) |
| `autonomousPeriodic()` | Every 20ms in auto | Usually empty | Yes |
| `teleopInit()` | Once per teleop | Cancel auto | Yes |
| `teleopPeriodic()` | Every 20ms in teleop | Usually empty | Yes |

## Key Takeaways

1. **Main → Robot → RobotContainer** - initialization chain
2. **Robot mode** determines which init/periodic methods run
3. **robotPeriodic() always runs** - 50Hz regardless of mode
4. **CommandScheduler.run() is the engine** - runs in robotPeriodic()
5. **All subsystems created in constructor** - not lazily
6. **IO layer selected by mode** - Real vs Sim vs Replay
7. **Logging happens in periodic** - every subsystem logs every cycle

## What's Next?

In **Part 8: Libraries & Advanced Topics**, we'll explore:
- Custom library code in `lib/`
- BeamBreak sensor wrapper
- SamuraiXboxController enhancements
- Drive system with clutches
- Odometry and pose estimation basics
- Best practices and next steps

Let's dive into the advanced features and custom utilities!

---

**Navigation:** [← Part 6](06-commands-and-bindings.md) | [Part 8 →](08-libraries-and-advanced.md)
