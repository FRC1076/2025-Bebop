# Part 3: Basic Subsystems Deep Dive

Now that you understand the IO Layer pattern, let's explore how subsystems work. We'll start with the two simplest subsystems: **Intake** and **Index** (the conveyor that moves notes through the robot).

## What is a Subsystem?

In WPILib's Command-Based framework, a **subsystem** is:

1. A class that extends `SubsystemBase`
2. Represents a physical mechanism on the robot
3. Has a `periodic()` method that runs every 20ms
4. Can only have ONE command using it at a time (prevents conflicts)
5. Provides methods to control the mechanism
6. Can create Command factory methods

Think of subsystems as the **API for your robot's mechanisms**.

## Anatomy of a Simple Subsystem

Let's dissect the IntakeSubsystem line by line.

### The Complete Intake Subsystem

**File:** [IntakeSubsystem.java](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java)

```java
public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.setVoltage(0);
    }

    public Command runVolts(double volts) {
        return Commands.run(
            () -> setVoltage(volts),
            this
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
```

### Line-by-Line Breakdown

#### Class Declaration
```java
public class IntakeSubsystem extends SubsystemBase {
```
- Extends `SubsystemBase` from WPILib
- Gets access to the command scheduler
- Provides the `periodic()` method hook

#### Fields
```java
private final IntakeIO io;
private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
```
- `io`: The interface to hardware (could be real or disabled)
- `inputs`: The auto-logged container for sensor data
- Both are `final` - they never change after construction

#### Constructor
```java
public IntakeSubsystem(IntakeIO io) {
    this.io = io;
}
```
- Receives the IO implementation via dependency injection
- Doesn't know if it's hardware or simulation
- Simple and testable!

#### Control Methods
```java
public void setVoltage(double volts) {
    io.setVoltage(volts);
}

public void stop() {
    io.setVoltage(0);
}
```
- **`setVoltage()`**: The basic control primitive
- **`stop()`**: Convenience method (just sets voltage to 0)
- Both delegate to the IO layer

#### Command Factory
```java
public Command runVolts(double volts) {
    return Commands.run(
        () -> setVoltage(volts),
        this
    );
}
```
- Returns a `Command` that runs the intake continuously
- `Commands.run()` creates a command that runs a lambda repeatedly
- `this` tells the scheduler "this command requires the IntakeSubsystem"
- Command will keep running until interrupted

**How it's used:**
```java
// In Superstructure or RobotContainer
intake.runVolts(10).schedule();  // Start running at 10V
```

#### Periodic Method
```java
@Override
public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
}
```
- Called every 20ms by the command scheduler
- **ALWAYS** updates inputs first
- **ALWAYS** logs inputs second
- This pattern is identical in every subsystem!

## The Index Subsystem

**File:** [IndexSubsystem.java](src/main/java/frc/robot/subsystems/index/IndexSubsystem.java)

The Index subsystem is almost identical to Intake! It's a simple roller that conveys notes.

```java
public class IndexSubsystem extends SubsystemBase {
    private final IndexIO io;
    private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

    public IndexSubsystem(IndexIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.setVoltage(0);
    }

    public Command runVolts(double volts) {
        return Commands.run(
            () -> setVoltage(volts),
            this
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Index", inputs);
    }
}
```

**The only differences:**
- Class name
- IO interface type
- Logger name

This shows the power of the IO Layer pattern - subsystems are simple!

## Connecting to Constants

Where do those voltage values come from? Let's trace it!

### Constants for Intake

**File:** [Constants.java:30-34](src/main/java/frc/robot/Constants.java#L30-L34)

```java
public static class IntakeConstants {
    public static final int kIntakeMotorCANId = 5;
    public static final boolean kIntakeMotorInverted = false;
    public static final int kIntakeMotorCurrentLimit = 20;
}
```

These are used in the hardware implementation:

**File:** [IntakeIOHardware.java](src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java) (simplified)

```java
public class IntakeIOHardware implements IntakeIO {
    private final CANSparkMax motor;

    public IntakeIOHardware() {
        motor = new CANSparkMax(
            IntakeConstants.kIntakeMotorCANId,      // CAN ID: 5
            MotorType.kBrushless
        );

        motor.setInverted(IntakeConstants.kIntakeMotorInverted);  // false
        motor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);  // 20A
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.velocityRadPerSec = motor.getEncoder().getVelocity() * (2 * Math.PI) / 60.0;
    }
}
```

### Constants for Index

**File:** [Constants.java:69-73](src/main/java/frc/robot/Constants.java#L69-L73)

```java
public static class IndexConstants {
    public static final int kIndexMotorCANId = 61;
    public static final boolean kIndexMotorInverted = false;
    public static final int kIndexMotorCurrentLimit = 20;
}
```

**Pattern:** All hardware configuration lives in `Constants.java`, not scattered through code!

## Voltage Control Explained

Both Intake and Index use **voltage control** - the simplest motor control strategy.

### What is Voltage Control?

```
You say: "Run at 10 volts"
Motor does: Spins at whatever speed 10V gives it
```

**Characteristics:**
- Simple and direct
- Speed varies with battery voltage (12.5V battery ≠ 11.0V battery)
- Speed varies with load (harder to spin = slower)
- No feedback loop

**When to use it:**
- Mechanisms where precise speed doesn't matter
- Rollers, conveyors, intakes
- Simple actuators

**When NOT to use it:**
- Precise positioning (use PID)
- Consistent speed under varying load (use PID)

### Voltage Examples

```java
// Full power forward
intake.setVoltage(12.0);

// Half power forward
intake.setVoltage(6.0);

// Full power backward
intake.setVoltage(-12.0);

// Stop
intake.setVoltage(0);
```

## Subsystem Requirements from CommandScheduler

The WPILib CommandScheduler enforces these rules:

### Rule 1: Only One Command per Subsystem
```java
Command cmd1 = intake.runVolts(10);
Command cmd2 = intake.runVolts(-5);

cmd1.schedule();  // Intake runs at 10V
cmd2.schedule();  // cmd1 is CANCELLED, intake now runs at -5V
```

This prevents conflicts like trying to move the same mechanism in two directions!

### Rule 2: Commands Declare Requirements
```java
public Command runVolts(double volts) {
    return Commands.run(
        () -> setVoltage(volts),
        this  // ← Declares "I require IntakeSubsystem"
    );
}
```

Without the `this`, the scheduler won't know to cancel conflicting commands!

### Rule 3: Periodic Always Runs
```java
public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
}
```

This runs EVERY 20ms, whether a command is scheduled or not. It's for:
- Updating sensor readings
- Logging data
- Safety checks
- Monitoring state

## How Subsystems Get Used

Let's trace a complete example: **Pressing the left trigger to intake**

### Step 1: Button Binding
**File:** [RobotContainer.java:139-140](src/main/java/frc/robot/RobotContainer.java#L139-L140)

```java
m_driverController.leftTrigger()
    .whileTrue(superstructureCommands.intake());
```

"While left trigger is pressed, run the intake command"

### Step 2: Superstructure Command
**File:** [Superstructure.java:219-225](src/main/java/frc/robot/subsystems/Superstructure.java#L219-L225)

```java
public Command intake() {
    return Commands.sequence(
        applyStateArmFirst(MechanismState.INTAKE),
        Commands.waitUntil(superstructure.superState.hasNote),
        applyStateAllParallel(MechanismState.SUBWOOFER)
    );
}
```

This command:
1. Applies INTAKE state (arm down, rollers on)
2. Waits for note detection
3. Transitions to SUBWOOFER state

### Step 3: Applying State
**File:** [Superstructure.java:148-160](src/main/java/frc/robot/subsystems/Superstructure.java#L148-L160)

```java
private Command applyStateArmFirst(MechanismState state) {
    superState.setMechanismState(state);

    return Commands.sequence(
        m_arm.startPid(state.armPositionRadians),
        Commands.waitUntil(() -> m_arm.withinTolerance(ArmConstants.kToleranceRadians)),
        Commands.parallel(
            m_intake.runVolts(state.intakeVolts),
            m_index.runVolts(state.indexVolts),
            m_shooter.startPid(state.shooterLeftSpeedRadPerSec, state.shooterRightSpeedRadPerSec)
        )
    );
}
```

### Step 4: State Values
**File:** [Constants.java:198-204](src/main/java/frc/robot/Constants.java#L198-L204)

```java
INTAKE(
    10,    // intakeVolts - run intake at 10V
    4,     // indexVolts - run index at 4V
    -0.6457718,  // armPositionRadians - arm down
    0,     // shooterLeftSpeedRadPerSec - shooter off
    0      // shooterRightSpeedRadPerSec - shooter off
),
```

### Step 5: Execution
```
m_intake.runVolts(10)  →  Intake motor runs at 10V
m_index.runVolts(4)    →  Index motor runs at 4V
```

Every 20ms:
- `IntakeSubsystem.periodic()` reads motor current, voltage, speed
- `IndexSubsystem.periodic()` reads motor current, voltage, speed
- All data gets logged to NetworkTables and log files

## Try It Yourself

### Exercise 1: Find the Index CAN ID
1. Open [Constants.java](src/main/java/frc/robot/Constants.java)
2. Find `IndexConstants`
3. What CAN ID is the index motor on?
4. What is its current limit?

### Exercise 2: Trace the Intake Hardware Setup
1. Open [IntakeIOHardware.java](src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java)
2. In the constructor, what type of motor is created? (Brushed or brushless?)
3. Is the motor inverted?
4. What idle mode is set? (Brake or coast?)

### Exercise 3: Understand the Command Factory
1. Look at `IntakeSubsystem.runVolts()` in [IntakeSubsystem.java:32-37](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java#L32-L37)
2. What method does the command call repeatedly?
3. What subsystem does it require?
4. Will this command end on its own, or must it be interrupted?

### Exercise 4: Calculate Intake Speed
In [IntakeIOHardware.java](src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java), find the velocity calculation:
```java
inputs.velocityRadPerSec = motor.getEncoder().getVelocity() * (2 * Math.PI) / 60.0;
```
1. What units does `motor.getEncoder().getVelocity()` return? (Hint: Spark MAX default)
2. Why multiply by `2 * Math.PI`?
3. Why divide by 60?
4. What are the final units?

### Exercise 5: Design Your Own Simple Subsystem
On paper, design a subsystem for a simple roller mechanism:
1. What would be in your `RollerIO` interface?
2. What constants would you need in `RollerConstants`?
3. What methods would `RollerSubsystem` have?
4. Would you use voltage control or PID control? Why?

## Common Patterns in Simple Subsystems

### Pattern 1: The Standard Periodic
```java
@Override
public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("SubsystemName", inputs);
}
```
**Every** subsystem should have this!

### Pattern 2: Basic Control Method
```java
public void setVoltage(double volts) {
    io.setVoltage(volts);
}
```
Thin wrapper around IO - subsystem adds no logic (yet).

### Pattern 3: Convenience Stop
```java
public void stop() {
    io.setVoltage(0);
}
```
Makes code clearer than `setVoltage(0)`.

### Pattern 4: Command Factory
```java
public Command runVolts(double volts) {
    return Commands.run(() -> setVoltage(volts), this);
}
```
Returns a command that can be scheduled, composed, bound to buttons.

## What Makes a Subsystem "Simple"?

A simple subsystem:
- ✅ Has one or two motors
- ✅ Uses voltage control (not PID)
- ✅ Has minimal state
- ✅ Has no complex logic in control methods
- ✅ Delegates everything to IO layer

Examples in this codebase:
- IntakeSubsystem
- IndexSubsystem

**Next level up:** Subsystems with PID control, feedforward, state machines (we'll cover these in Part 4!)

## Key Takeaways

1. **Subsystems extend SubsystemBase** - gives you scheduling and lifecycle hooks
2. **IO layer does hardware** - subsystems do logic
3. **Periodic updates and logs** - runs every 20ms, regardless of commands
4. **Command factories** - subsystems create commands that control them
5. **One command at a time** - scheduler prevents conflicts
6. **Constants are centralized** - all config in `Constants.java`
7. **Voltage control is simple** - no feedback, just open-loop control

## What's Next?

In **Part 4: Advanced Subsystems**, we'll explore:
- PID control for precise positioning (Arm)
- Velocity control with PID (Shooter)
- Feedforward control
- Swerve drive modules
- Software limits and safety

We'll see how subsystems can be much more complex while still following the same patterns!

---

**Navigation:** [← Part 2](02-understanding-io-layers.md) | [Part 4 →](04-advanced-subsystems.md)
