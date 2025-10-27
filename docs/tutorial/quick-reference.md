# Quick Reference Sheet

A fast lookup guide for common patterns, file locations, and code snippets from the 2025-Bebop robot.

## File Locations

### Core Robot Files
| File | Purpose | Key Contents |
|------|---------|--------------|
| [Main.java](src/main/java/frc/robot/Main.java) | Entry point | `main()` method |
| [Robot.java](src/main/java/frc/robot/Robot.java) | Robot lifecycle | AdvantageKit setup, mode handlers |
| [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) | Setup & bindings | Creates subsystems, binds buttons |
| [Constants.java](src/main/java/frc/robot/Constants.java) | All configuration | Motor IDs, PID gains, positions, speeds |

### Subsystems
| Subsystem | Location | Type |
|-----------|----------|------|
| Arm | [src/main/java/frc/robot/subsystems/arm/](src/main/java/frc/robot/subsystems/arm/) | Position control (PID) |
| Intake | [src/main/java/frc/robot/subsystems/intake/](src/main/java/frc/robot/subsystems/intake/) | Voltage control |
| Index | [src/main/java/frc/robot/subsystems/index/](src/main/java/frc/robot/subsystems/index/) | Voltage control |
| Shooter | [src/main/java/frc/robot/subsystems/shooter/](src/main/java/frc/robot/subsystems/shooter/) | Velocity control (PID) |
| Drive | [src/main/java/frc/robot/subsystems/drive/](src/main/java/frc/robot/subsystems/drive/) | Swerve drive |
| Superstructure | [Superstructure.java](src/main/java/frc/robot/subsystems/Superstructure.java) | State machine coordinator |

### IO Layer Pattern
Each subsystem has three files:
- `XXXSubsystem.java` - Business logic
- `XXXIO.java` - Interface (defines inputs/outputs)
- `XXXIOHardware.java` - Real hardware implementation
- `XXXIODisabled.java` - Simulation/disabled implementation

## Common Constants

### Motor CAN IDs
```java
// From Constants.java
Intake Motor:        5
Arm Lead Motor:      6
Arm Follow Motor:    7
Shooter Left Motor:  28
Shooter Right Motor: 8
Index Motor:         61

// Drive modules (per module: drive, turn, encoder)
Front Left:   1,  11, 21
Front Right:  2,  12, 22
Rear Left:    4,  14, 24
Rear Right:   3,  13, 23

Gyro (if Pigeon): 9
Beam Break DIO:   5
```

### Key Positions (Radians)
```java
// From Constants.java - MechanismState enum
HOME Arm:      -0.6457718
INTAKE Arm:    -0.6457718
SUBWOOFER Arm: -0.4014257
MID_LOW Arm:    0.0
MID_HIGH Arm:   0.5
AMP Arm:        1.3
```

### Shooter Speeds (rad/s)
```java
SUBWOOFER:   Left: -471,  Right: 576
MID_LOW:     Left: -400,  Right: 489
MID_HIGH:    Left: -350,  Right: 428
AMP:         Left: -250,  Right: 280
```

## Code Patterns

### Creating a Simple Subsystem

```java
public class MySubsystem extends SubsystemBase {
    private final MyIO io;
    private final MyIOInputsAutoLogged inputs = new MyIOInputsAutoLogged();

    public MySubsystem(MyIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public Command runVolts(double volts) {
        return Commands.run(() -> setVoltage(volts), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("MySubsystem", inputs);
    }
}
```

### Creating an IO Interface

```java
public interface MyIO {
    @AutoLog
    public static class MyIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double velocityRadPerSec = 0;
    }

    public abstract void setVoltage(double volts);
    public abstract void updateInputs(MyIOInputs inputs);
}
```

### Creating a Hardware IO

```java
public class MyIOHardware implements MyIO {
    private final CANSparkMax motor;

    public MyIOHardware() {
        motor = new CANSparkMax(
            Constants.MyConstants.kMotorCANId,
            MotorType.kBrushless
        );
        motor.setInverted(Constants.MyConstants.kMotorInverted);
        motor.setSmartCurrentLimit(Constants.MyConstants.kCurrentLimit);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(MyIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.velocityRadPerSec = motor.getEncoder().getVelocity() * (2 * Math.PI) / 60.0;
    }
}
```

### Creating a Disabled IO

```java
public class MyIODisabled implements MyIO {
    public MyIODisabled() {}

    @Override
    public void setVoltage(double volts) {}

    @Override
    public void updateInputs(MyIOInputs inputs) {}
}
```

### Command Composition Examples

```java
// Sequential (one after another)
Commands.sequence(
    command1,
    command2,
    command3
)

// Parallel (all at once)
Commands.parallel(
    command1,
    command2,
    command3
)

// Race (first to finish cancels others)
Commands.race(
    runForever,
    Commands.waitSeconds(3)
)

// Deadline (main command determines end)
Commands.deadline(
    Commands.waitSeconds(3),  // When this ends, all end
    command1,
    command2
)

// Wait for condition
Commands.waitUntil(() -> sensor.isTriggered())

// Run once then finish
Commands.runOnce(() -> doSomething())

// Run repeatedly forever
Commands.run(() -> doSomething(), subsystem)
```

### Button Binding Examples

```java
// Runs while button held, cancels when released
controller.a()
    .whileTrue(command);

// Runs when pressed, continues until finished (even if released)
controller.b()
    .onTrue(command);

// Runs when released
controller.x()
    .onFalse(command);

// First press starts, second press stops
controller.y()
    .toggleOnTrue(command);

// Chaining
controller.leftBumper()
    .whileTrue(slowMode)
    .onFalse(normalMode);

// Compound triggers
controller.a().and(controller.b())
    .onTrue(command);

controller.a().or(controller.b())
    .onTrue(command);

controller.a().negate()
    .whileTrue(command);  // While NOT pressed
```

### PID Setup Pattern

```java
public class MySubsystem extends SubsystemBase {
    private ProfiledPIDController pidController;
    private boolean runPID = false;
    private double pidTarget = 0;

    public MySubsystem(MyIO io) {
        this.io = io;

        pidController = new ProfiledPIDController(
            kP, kI, kD,
            new Constraints(kMaxVelocity, kMaxAcceleration)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (runPID) {
            double output = pidController.calculate(inputs.position, pidTarget);
            setVoltage(output);
        }

        Logger.processInputs("MySubsystem", inputs);
    }

    public Command startPid(double target) {
        return Commands.sequence(
            Commands.runOnce(() -> pidController.reset(inputs.position)),
            Commands.runOnce(() -> pidTarget = target),
            Commands.runOnce(() -> runPID = true)
        );
    }

    public boolean atTarget() {
        return Math.abs(pidTarget - inputs.position) < kTolerance;
    }
}
```

## Controller Bindings (Current Robot)

### Xbox Controller Layout
```
Left Trigger:     Intake
Right Trigger:    Shoot

A Button:         Go to SUBWOOFER position
B Button:         Go to MID_LOW position
X Button:         Go to MID_HIGH position
Y Button:         Go to AMP position

D-Pad Up:         Arm up (manual)
D-Pad Down:       Arm down (manual)
D-Pad Left:       All rollers backward (manual)
D-Pad Right:      All rollers forward (manual)

Right Bumper:     Single clutch (60% speed)
Left Bumper:      Double clutch (35% speed)

Start Button:     Re-zero gyro

Left Stick:       Drive (X/Y translation)
Right Stick X:    Rotation
```

## State Machine Quick Reference

### State Flow
```
Power On → HOME

Driver holds left trigger:
  HOME → INTAKE → (detects note) → SUBWOOFER

Driver presses A/B/X/Y:
  SUBWOOFER → (respective position)

Driver holds right trigger:
  (current position) → SHOOT_XXX → (note gone) → HOME
```

### All States
1. **HOME** - Default, arm down, everything off
2. **INTAKE** - Arm down, rollers on, grabbing note
3. **SUBWOOFER** - Low angle, shooter spinning, ready to shoot close
4. **MID_LOW** - Medium low angle, ready to shoot
5. **MID_HIGH** - Medium high angle, ready to shoot
6. **AMP** - High angle, ready to score in amp
7. **SHOOT_SUBWOOFER** - Actually shooting from subwoofer
8. **SHOOT_MID_LOW** - Actually shooting from mid-low
9. **SHOOT_MID_HIGH** - Actually shooting from mid-high
10. **SHOOT_AMP** - Actually shooting into amp

## Common Gradle Commands

```bash
# Build the project
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulator
./gradlew simulateJava

# Run tests
./gradlew test

# Clean build files
./gradlew clean
```

## Debugging Tips

### Check These First
1. Are motors configured correctly? (CAN IDs, inversions)
2. Is the IO layer wired correctly in RobotContainer?
3. Are button bindings correct?
4. Are PID gains tuned? (Many are still 0!)
5. Is the mode set correctly? (REAL vs SIM vs REPLAY)

### Using Logs
1. Check NetworkTables in AdvantageScope
2. Look for `SuperState/MechanismState` to see state
3. Check `Arm/PositionRadians` vs `Arm/PidTargetRadians`
4. Monitor motor currents for binding/stalling

### Common Errors
| Error | Likely Cause | Fix |
|-------|--------------|-----|
| Command doesn't run | Missing requirement | Add subsystem to `Commands.run(..., subsystem)` |
| Motors don't move | Wrong mode (SIM vs REAL) | Check `SystemConstants.currentMode` |
| PID oscillates | Gains too high | Reduce kP, increase kD |
| PID too slow | Gains too low | Increase kP |
| Button does nothing | Binding not in `configureDriverBindings()` | Add binding |
| Subsystem conflicts | Two commands need same subsystem | Check requirements |

## Useful WPILib Classes

| Class | Purpose | Example |
|-------|---------|---------|
| `Commands.sequence()` | Run commands in order | See Superstructure |
| `Commands.parallel()` | Run commands together | See Superstructure |
| `Commands.run()` | Repeating command | `Commands.run(() -> intake.setVoltage(10), intake)` |
| `Commands.runOnce()` | One-shot command | `Commands.runOnce(() -> arm.reset())` |
| `Commands.waitSeconds()` | Delay | `Commands.waitSeconds(2.0)` |
| `Commands.waitUntil()` | Wait for condition | `Commands.waitUntil(() -> arm.atTarget())` |
| `Timer` | Timing | `timer.hasElapsed(3.0)` |
| `MathUtil.applyDeadband()` | Input filtering | Used in SamuraiXboxController |

## Learning Resources

- **WPILib Docs**: https://docs.wpilib.org/
- **AdvantageKit**: https://github.com/Mechanical-Advantage/AdvantageKit
- **Chief Delphi**: https://www.chiefdelphi.com/
- **FRC Discord**: https://discord.gg/frc

---

**Navigation:** [← Part 8](08-libraries-and-advanced.md) | [Glossary →](glossary.md)
