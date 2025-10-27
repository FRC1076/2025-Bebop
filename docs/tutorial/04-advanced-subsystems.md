# Part 4: Advanced Subsystems

Now let's explore subsystems that use **closed-loop control** - they use feedback to achieve precise positions or velocities. We'll examine the **Arm** (position control) and **Shooter** (velocity control) subsystems.

## Open-Loop vs Closed-Loop Control

### Open-Loop (What We've Seen)
```
You: "Run at 10 volts"
Motor: *spins*
You: *don't check what happens*
```

**Problem:** No feedback! You don't know if it's actually doing what you want.

### Closed-Loop (This Chapter)
```
You: "Go to 45 degrees"
Motor: *spins*
Sensor: "Currently at 30 degrees"
Controller: "Need 15 more degrees, increase power"
Motor: *spins faster*
Sensor: "Currently at 44 degrees"
Controller: "Almost there, reduce power"
Motor: *slows down*
Sensor: "At 45 degrees!"
Controller: "Stop, we're there"
```

**Benefit:** Accurate and repeatable, regardless of battery voltage or load!

## PID Control Fundamentals

**PID** stands for **Proportional-Integral-Derivative** - a feedback control algorithm.

### The Components

Given an error (desired position - actual position):

1. **P (Proportional)**: Output proportional to error
   - Big error → big response
   - Small error → small response
   - Formula: `P = kP * error`

2. **I (Integral)**: Accumulates error over time
   - Eliminates steady-state error
   - Helps overcome friction/gravity
   - Formula: `I = kI * Σ(error)`

3. **D (Derivative)**: Responds to rate of change
   - Slows down as you approach target
   - Reduces overshoot
   - Formula: `D = kD * (error - previousError)`

**Total output:** `output = P + I + D`

### Simple Example

Target: 90°, Current: 0°
```
Error = 90° - 0° = 90°

P = 0.1 * 90 = 9.0 volts
I = 0 (no accumulated error yet)
D = 0 (no change in error yet)

Output = 9.0 volts → arm moves up
```

Next cycle: Current: 45°
```
Error = 90° - 45° = 45°

P = 0.1 * 45 = 4.5 volts
I = 0.01 * (90 + 45) = 1.35 volts (accumulated)
D = 0.05 * (45 - 90) = -2.25 volts (slowing down)

Output = 4.5 + 1.35 - 2.25 = 3.6 volts → arm continues but slower
```

## The Arm Subsystem: Position Control

The arm needs to move to precise angles for shooting at different distances.

### Arm Architecture

**File:** [ArmSubsystem.java](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java)

```java
public class ArmSubsystem extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private ProfiledPIDController pidController;
    private boolean runPID = false;
    private double PIDTargetRadians = ArmConstants.kAbsoluteEncoderZero;

    private ArmFeedforward feedForwardController;

    public ArmSubsystem(ArmIO io) {
        this.io = io;

        pidController = new ProfiledPIDController(
            Control.kP,
            Control.kI,
            Control.kD,
            new Constraints(Control.kMaxVelocity, Control.kMaxAcceleration)
        );

        feedForwardController = new ArmFeedforward(
            Control.kS,
            Control.kG,
            Control.kV
        );
    }
}
```

### Key Differences from Simple Subsystems

1. **`ProfiledPIDController`** instead of just PID
   - Limits velocity and acceleration
   - Creates smooth motion profiles
   - Prevents jerky movement

2. **`ArmFeedforward`** controller
   - Predicts voltage needed based on physics
   - Helps PID work better
   - Compensates for gravity!

3. **State variables**
   - `runPID`: Is PID active?
   - `PIDTargetRadians`: Where should we go?

### The Constants

**File:** [Constants.java:54-66](src/main/java/frc/robot/Constants.java#L54-L66)

```java
public static class Control {
    // PID gains (TO BE DETERMINED FROM TUNING)
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // Feedforward gains
    public static final double kS = 0;  // Voltage to overcome static friction
    public static final double kG = 0;  // Voltage to hold against gravity
    public static final double kV = 0;  // Voltage per unit velocity

    // Motion constraints
    public static final double kMaxVelocity = 0;      // rad/s
    public static final double kMaxAcceleration = 0;  // rad/s²
}
```

**Note:** These are currently 0 - they need to be tuned through testing!

### Software Limits: Safety First!

**File:** [ArmSubsystem.java:50-58](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java#L50-L58)

```java
public void setVoltage(double volts) {
    if (inputs.positionRadians > ArmConstants.kMaxPositionRadians && volts > 0) {
        volts = 0;  // Don't go further up!
    } else if (inputs.positionRadians < ArmConstants.kMinPositionRadians && volts < 0) {
        volts = 0;  // Don't go further down!
    }

    io.setVoltage(volts + feedForwardController.calculate(inputs.positionRadians, inputs.velocityRadiansPerSecond));
}
```

**This prevents:**
- Arm hitting the robot frame
- Arm crashing into the ground
- Mechanical damage

**Note the feedforward addition:** The actual voltage sent is `commandedVoltage + feedforwardVoltage`

### The Periodic Method: Running PID

**File:** [ArmSubsystem.java:84-96](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java#L84-L96)

```java
@Override
public void periodic() {
    io.updateInputs(inputs);

    inputs.pidRunning = runPID;

    if (runPID) {
        setVoltage(pidController.calculate(inputs.positionRadians, PIDTargetRadians));
        inputs.pidTargetRadians = PIDTargetRadians;
    }

    Logger.processInputs("Arm", inputs);
}
```

**Flow:**
1. Read sensors (`updateInputs`)
2. If PID is enabled:
   - Calculate PID output based on current position
   - Set voltage to that output (which adds feedforward)
   - Log the target for debugging
3. Log everything

**Key insight:** PID runs in `periodic()` (every 20ms), not in a command!

### Command Factory: Starting PID

**File:** [ArmSubsystem.java:102-108](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java#L102-L108)

```java
public Command startPid(double goalRadians) {
    return Commands.sequence(
        Commands.runOnce(() -> pidController.reset(inputs.positionRadians)),
        Commands.runOnce(() -> setPidTarget(goalRadians)),
        Commands.runOnce(() -> setRunPid(true))
    );
}
```

This command:
1. **Resets PID controller** - clears accumulated error (I term)
2. **Sets the target** - where we want to go
3. **Enables PID** - turns on the control loop

**Important:** This command finishes instantly! The PID continues running in `periodic()`.

### Checking if We're There

**File:** [ArmSubsystem.java:79-81](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java#L79-L81)

```java
public boolean withinTolerance(double toleranceRadians) {
    return Math.abs(PIDTargetRadians - inputs.positionRadians) < toleranceRadians;
}
```

Used in commands:
```java
Commands.waitUntil(() -> m_arm.withinTolerance(ArmConstants.kToleranceRadians))
```

This lets other subsystems wait for the arm before proceeding!

## The Shooter Subsystem: Velocity Control

The shooter spins flywheels to precise speeds to launch notes accurately.

### Shooter Architecture

**File:** [ShooterSubsystem.java](src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java)

```java
public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private PIDController leftPidController;
    private PIDController rightPidController;
    private double leftPidTargetRadPerSec = 0;
    private double rightPidTargetRadPerSec = 0;
    private boolean runPid = false;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        leftPidController = new PIDController(
            Control.kPLeft,
            Control.kILeft,
            Control.kDLeft
        );

        rightPidController = new PIDController(
            Control.kPRight,
            Control.kIRight,
            Control.kDRight
        );
    }
}
```

### Key Differences from Arm

1. **Two motors** = two PID controllers
   - Left and right wheels can spin at different speeds
   - Enables backspin/topspin by differential speeds

2. **Regular `PIDController`** not `ProfiledPIDController`
   - Don't need motion profiling for velocity
   - Just need to hit a target speed

3. **No feedforward** (yet)
   - Could add later for faster response
   - PID alone often sufficient for flywheels

### Dual Motor Control

**File:** [ShooterSubsystem.java:67-79](src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java#L67-L79)

```java
@Override
public void periodic() {
    io.updateInputs(inputs);

    inputs.pidRunning = runPid;
    if (runPid) {
        inputs.leftMotorPidTargetRadPerSec = leftPidTargetRadPerSec;
        inputs.rightMotorPidTargetRadPerSec = rightPidTargetRadPerSec;

        leftPidController.calculate(inputs.leftEncoderVelocityRadPerSec, leftPidTargetRadPerSec);
        rightPidController.calculate(inputs.rightEncoderVelocityRadPerSec, rightPidTargetRadPerSec);
    }

    Logger.processInputs("Shooter", inputs);
}
```

**Two separate control loops:**
- Left PID: `currentLeftSpeed` → `targetLeftSpeed`
- Right PID: `currentRightSpeed` → `targetRightSpeed`

This gives independent control for spin effects!

### Starting the Shooter

**File:** [ShooterSubsystem.java:87-92](src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java#L87-L92)

```java
public Command startPid(double leftMotorTargetRadPerSec, double rightMotorTargetRadPerSec) {
    return Commands.sequence(
        Commands.runOnce(() -> setPIDTarget(leftMotorTargetRadPerSec, rightMotorTargetRadPerSec)),
        Commands.runOnce(() -> setRunPID(true))
    );
}
```

Similar to arm, but:
- No reset needed (velocity control doesn't accumulate error the same way)
- Two targets instead of one

## Feedforward: Physics-Based Control

Feedforward predicts the voltage needed based on the physics model of your mechanism.

### For Arms (ArmFeedforward)

```java
feedForwardController = new ArmFeedforward(kS, kG, kV);

voltage = kS * sign(velocity)           // Overcome friction
        + kG * cos(angle)               // Overcome gravity
        + kV * velocity;                // Overcome inertia
```

**Components:**
- **kS**: Static friction - voltage to start moving
- **kG**: Gravity compensation - holds arm horizontal
- **kV**: Velocity gain - voltage per rad/s

**Why it helps:**
- PID only corrects errors
- Feedforward prevents errors from happening
- Faster response, less overshoot

### For Flywheels (SimpleMotorFeedforward)

```java
feedforward = new SimpleMotorFeedforward(kS, kV, kA);

voltage = kS * sign(velocity)      // Overcome friction
        + kV * velocity            // Maintain speed
        + kA * acceleration;       // Change speed
```

This codebase doesn't use shooter feedforward yet, but could add it!

## Position vs Velocity Control Comparison

| Aspect | Position (Arm) | Velocity (Shooter) |
|--------|----------------|-------------------|
| **Goal** | Reach and hold angle | Maintain speed |
| **Sensor** | Absolute encoder | Velocity from encoder |
| **Units** | Radians | Radians per second |
| **Typical Use** | Arms, elevators, turrets | Flywheels, rollers |
| **At Target** | Holds position | Keeps spinning |
| **Feedforward** | Gravity compensation | Speed compensation |
| **Profile?** | Yes (smooth motion) | No (just hit speed) |

## How It All Connects: Shooting Sequence

Let's trace what happens when you shoot from the SUBWOOFER position:

### 1. You Press 'A' Button
**File:** [RobotContainer.java:147-148](src/main/java/frc/robot/RobotContainer.java#L147-L148)
```java
m_driverController.a()
    .onTrue(superstructureCommands.subwoofer());
```

### 2. Superstructure Positions for Subwoofer
**File:** [Superstructure.java:228-230](src/main/java/frc/robot/subsystems/Superstructure.java#L228-L230)
```java
public Command subwoofer() {
    return applyStateAllParallel(MechanismState.SUBWOOFER);
}
```

### 3. State Gets Applied
**File:** [Superstructure.java:168-177](src/main/java/frc/robot/subsystems/Superstructure.java#L168-L177)
```java
private Command applyStateAllParallel(MechanismState state) {
    superState.setMechanismState(state);

    return Commands.parallel(
        m_arm.startPid(state.armPositionRadians),
        m_intake.runVolts(state.intakeVolts),
        m_index.runVolts(state.indexVolts),
        m_shooter.startPid(state.shooterLeftSpeedRadPerSec, state.shooterRightSpeedRadPerSec)
    );
}
```

### 4. SUBWOOFER State Values
**File:** [Constants.java:207-212](src/main/java/frc/robot/Constants.java#L207-L212)
```java
SUBWOOFER(
    0,              // intakeVolts - intake off
    0,              // indexVolts - index off
    -0.4014257,     // armPositionRadians - arm angle for close shot
    -471,           // shooterLeftSpeedRadPerSec
    576             // shooterRightSpeedRadPerSec - note the difference!
),
```

### 5. Control Loops Engage

**Arm:**
```
Every 20ms:
  Current position: -0.5 rad
  Target: -0.4014257 rad
  Error: 0.0985743 rad
  PID calculates: 3.2V
  Feedforward adds: 1.8V (gravity compensation)
  Total: 5.0V sent to motors

  Arm moves up...

  Current position: -0.401 rad
  Error: 0.0004257 rad (very small!)
  PID: 0.1V
  Feedforward: 1.8V
  Total: 1.9V (just holding against gravity)
```

**Shooter:**
```
Every 20ms:
  Left wheel: 0 rad/s → target -471 rad/s
  Right wheel: 0 rad/s → target 576 rad/s

  PID pushes hard to accelerate...

  After 1 second:
  Left wheel: -470 rad/s (almost there!)
  Right wheel: 575 rad/s (almost there!)

  PID reduces power to maintain speed
```

### 6. You Press Right Trigger to Shoot
**File:** [RobotContainer.java:143-144](src/main/java/frc/robot/RobotContainer.java#L143-L144)
```java
m_driverController.rightTrigger()
    .whileTrue(superstructureCommands.shoot());
```

### 7. Shoot Command Executes
**File:** [Superstructure.java:249-260](src/main/java/frc/robot/subsystems/Superstructure.java#L249-L260)
```java
public Command shoot() {
    return Commands.sequence(
        applyStateNoArmMove(
            scoringStates.getOrDefault(superstructure.superState.getMechanismState(), MechanismState.SHOOT_MID_HIGH)
        ),
        Commands.waitUntil(
            new Trigger(superstructure.superState.hasNote).negate()
                .debounce(SuperstructureConstants.kArmMoveDebounceTimeAfterShoot)
        ),
        applyStateAllParallel(MechanismState.HOME)
    );
}
```

Since we're in SUBWOOFER state, it transitions to SHOOT_SUBWOOFER, which:
- Keeps arm at same angle
- Turns on index roller (feeds note into shooter)
- Shooter already spinning at speed!

**Note flies out!**

## Try It Yourself

### Exercise 1: Understand Arm Limits
1. Open [Constants.java:49-50](src/main/java/frc/robot/Constants.java#L49-L50)
2. What is `kMaxPositionRadians`?
3. What is `kMinPositionRadians`?
4. Convert these to degrees (multiply by 180/π)
5. Why do you think these limits exist?

### Exercise 2: Trace PID Execution
1. Open [ArmSubsystem.java](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java)
2. In `startPid()`, what happens first?
3. After `startPid()` finishes, where does PID actually run?
4. How often does it run?
5. When does it stop running?

### Exercise 3: Compare Shooter Speeds
1. Open [Constants.java](src/main/java/frc/robot/Constants.java)
2. Find all four shooting states (SHOOT_SUBWOOFER, SHOOT_MID_LOW, SHOOT_MID_HIGH, SHOOT_AMP)
3. Make a table of left and right speeds for each
4. Which state has the highest speeds?
5. Why might left and right speeds be different?

### Exercise 4: Calculate Flywheel RPM
The shooter speeds are in radians per second. Convert SUBWOOFER speeds to RPM:
```
Formula: RPM = (rad/s) * (60 / 2π)

Left: -471 rad/s = ? RPM
Right: 576 rad/s = ? RPM
```

### Exercise 5: Design a PID Strategy
On paper, design a control strategy for a hypothetical elevator subsystem:
1. Would you use position or velocity control? Why?
2. Would you need ProfiledPID or regular PID?
3. What feedforward components would you need?
4. What safety limits would you implement?

## Common PID Tuning Tips

### Starting Values
1. Set I and D to 0
2. Increase P until oscillation starts
3. Back off P by 50%
4. Add D to reduce overshoot
5. Add I only if steady-state error exists

### If It Oscillates
- Decrease P gain
- Increase D gain
- Check for mechanical slop

### If It's Too Slow
- Increase P gain
- Add feedforward
- Check motion constraints (max velocity/acceleration)

### If It Never Reaches Target
- Increase I gain (slowly!)
- Add feedforward
- Check for static friction

## Key Takeaways

1. **PID provides feedback control** - automatically corrects errors
2. **ProfiledPID for position** - smooth motion with velocity/acceleration limits
3. **Regular PID for velocity** - flywheels and rollers
4. **Feedforward helps PID** - predicts needed voltage based on physics
5. **Software limits for safety** - prevent mechanism damage
6. **PID runs in periodic()** - command just enables it
7. **Tolerance checking** - know when you've reached the goal

## What's Next?

In **Part 5: The Superstructure**, we'll explore:
- How all these subsystems coordinate
- The state machine implementation
- Command composition and sequencing
- The Superstructure command factory

This is where everything comes together into robot behavior!

---

**Navigation:** [← Part 3](03-subsystems-deep-dive.md) | [Part 5 →](05-superstructure-architecture.md)
