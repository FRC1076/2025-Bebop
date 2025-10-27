# Part 8: Libraries & Advanced Topics

Beyond the core robot code, this project includes custom utility libraries, advanced drive features, and architectural best practices. Let's explore the `lib/` package and advanced concepts.

## The lib/ Package

The `lib/` directory contains reusable code that could be used across multiple robot projects:

```
lib/
├── extendedcommands/      # Custom command types
├── functional/            # Functional interfaces (like TriConsumer)
├── hardware/              # Hardware wrapper classes
│   ├── hid/              # Human Interface Device wrappers
│   └── BeamBreak.java    # Beam break sensor wrapper
└── utils/                 # Utility functions
```

**Philosophy:** Code here should be:
- Robot-agnostic (not specific to 2025 game)
- Well-tested
- Documented
- Reusable

## BeamBreak: Sensor Wrapper

**File:** [BeamBreak.java](src/main/java/lib/hardware/BeamBreak.java)

### The Complete Class

```java
public class BeamBreak {
    DigitalInput sensor;

    public BeamBreak(int channel) {
        sensor = new DigitalInput(channel);
    }

    public DigitalInput getSensor() {
        return sensor;
    }

    public boolean isBeamBroken() {
        return !sensor.get();
    }

    public BooleanSupplier beamBrokenSupplier() {
        return () -> !sensor.get();
    }
}
```

### Why Wrap DigitalInput?

**Without wrapper:**
```java
DigitalInput beamBreak = new DigitalInput(5);
boolean hasNote = !beamBreak.get();  // Why ! ? Not intuitive!
```

**With wrapper:**
```java
BeamBreak beamBreak = new BeamBreak(5);
boolean hasNote = beamBreak.isBeamBroken();  // Clear!
```

### The Logic Inversion

```java
return !sensor.get();
```

**Why negate?**
- `DigitalInput.get()` returns `true` when circuit is open (no object)
- We want `true` when beam is broken (object present)
- The `!` inverts the logic for clearer semantics

### The Supplier Pattern

```java
public BooleanSupplier beamBrokenSupplier() {
    return () -> !sensor.get();
}
```

**Usage in RobotContainer:**
```java
m_beamBreak = new BeamBreak(OIConstants.kBeamBreakPin);

m_superstructure = new Superstructure(
    m_arm,
    m_index,
    m_intake,
    m_shooter,
    m_beamBreak.beamBrokenSupplier()  // ← Lambda that reads sensor
);
```

**Why a supplier?**
- Superstructure doesn't own the sensor
- Supplier gets called when needed (live data)
- Decouples sensor hardware from Superstructure logic

**In commands:**
```java
Commands.waitUntil(superstructure.superState.hasNote)
// Repeatedly calls hasNote() until true
```

## SamuraiXboxController: Enhanced Input

**File:** [SamuraiXboxController.java](src/main/java/lib/hardware/hid/SamuraiXboxController.java)

This extends WPILib's `CommandXboxController` with better defaults.

### What It Adds

#### 1. Automatic Deadbanding

```java
public class SamuraiXboxController extends CommandXboxController {
    private double stickDeadband;
    private DoubleSupplier leftStickX_DB;

    private void configSticks() {
        leftStickX_DB = () -> MathUtil.applyDeadband(super.getLeftX(), stickDeadband);
        // ... same for other axes
    }

    @Override
    public double getLeftX() {
        return leftStickX_DB.getAsDouble();  // Automatically deadbanded!
    }
}
```

**What is deadbanding?**
```
Without deadband:
  Stick at rest: 0.03 (controller drift)
  Robot creeps slowly

With 0.15 deadband:
  Input < 0.15 → output 0
  Input > 0.15 → scaled from 0.15-1.0 to 0.0-1.0
  No drift!
```

**Implementation:**
```java
MathUtil.applyDeadband(value, deadband)
// If |value| < deadband: return 0
// Else: scale value from (deadband, 1.0) to (0, 1.0)
```

#### 2. Configurable Trigger Threshold

```java
@Override
public Trigger leftTrigger() {
    return super.leftTrigger(triggerThreshold);
}
```

**Why?**
- Triggers are analog (0.0 to 1.0)
- Need a threshold to convert to boolean
- Default 0.7 means "70% pressed = activated"

#### 3. Fluent API (Builder Pattern)

```java
private final SamuraiXboxController m_driverController =
    new SamuraiXboxController(OIConstants.kDriverControllerPort)
        .withDeadband(OIConstants.kControllerDeadband)
        .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);
```

**The `withXxx()` methods return `this`:**
```java
public SamuraiXboxController withDeadband(double deadband) {
    stickDeadband = deadband;
    configSticks();
    return this;  // ← Allows chaining!
}
```

**Benefit:** Configuration reads like English, one line, clear!

## Drive System: Clutches

**File:** [RobotContainer.java:67-68](src/main/java/frc/robot/RobotContainer.java#L67-L68)

```java
private double translationalClutch = 1;
private double rotationalClutch = 1;
```

### What is a Clutch?

A **clutch** is a speed multiplier (0.0 to 1.0) that slows the robot for precision:

```java
// In DriveClosedLoopTeleop.execute()
ChassisSpeeds speeds = new ChassisSpeeds(
    xSpeed * maxSpeed * translationalClutch,  // ← Multiplied here
    ySpeed * maxSpeed * translationalClutch,
    omegaSpeed * maxRotationSpeed
);
```

### Single Clutch

**File:** [RobotContainer.java:183-191](src/main/java/frc/robot/RobotContainer.java#L183-L191)

```java
m_driverController.rightBumper()
    .whileTrue(Commands.run(() -> {
        translationalClutch = 0.6;
        rotationalClutch = 0.6;
    })).and(m_driverController.leftBumper()).negate()
    .onFalse(Commands.runOnce(() -> {
        translationalClutch = 1;
        rotationalClutch = 1;
    })).and(m_driverController.leftBumper()).negate();
```

**Translation:**
- While right bumper held (AND left bumper NOT held): 60% speed
- When released (AND left bumper NOT held): back to 100%

### Double Clutch

**File:** [RobotContainer.java:194-202](src/main/java/frc/robot/RobotContainer.java#L194-L202)

```java
m_driverController.leftBumper()
    .whileTrue(Commands.run(() -> {
        translationalClutch = 0.35;
        rotationalClutch = 0.35;
    }))
    .onFalse(Commands.runOnce(() -> {
        translationalClutch = 1;
        rotationalClutch = 1;
    })).and(m_driverController.rightBumper()).negate();
```

**Translation:**
- While left bumper held: 35% speed (super slow!)
- When released (AND right bumper NOT held): back to 100%

### Why Two Clutches?

**Use cases:**
- **No clutch (1.0):** Fast movement, crossing field
- **Single clutch (0.6):** Moderate precision, approaching game pieces
- **Double clutch (0.35):** Maximum precision, scoring, fine positioning

**Priority:** Left bumper (double) overrides right bumper (single) due to `.and().negate()` logic

## Advanced Drive Concepts

### Swerve Drive Basics

This robot uses **swerve drive** - each wheel can:
1. Spin (drive motor)
2. Rotate (turn motor)

**Advantages:**
- Omnidirectional movement
- Can translate any direction while facing any direction
- Strafe, crab, rotate in place

**Complexity:**
- 8 motors total (4 modules × 2 motors each)
- Requires odometry (position tracking)
- Complex kinematics math

### Field-Oriented vs Robot-Oriented

**Robot-oriented:**
- "Forward" is robot's front
- When robot rotates, control directions change
- Harder for driver

**Field-oriented:**
- "Forward" is always downfield (from driver's perspective)
- Robot can spin while maintaining translation direction
- Much easier for driver!

**Implementation:** [DriveClosedLoopTeleop.java:55-59](src/main/java/frc/robot/commands/DriveClosedLoopTeleop.java#L55-L59)

```java
ChassisSpeeds speeds = new ChassisSpeeds(
    scaleSpeed(xTransSpeedSupplier.getAsDouble()) * maxSpeed * clutch,
    scaleSpeed(yTransSpeedSupplier.getAsDouble()) * maxSpeed * clutch,
    omegaSupplier.getAsDouble() * maxRotationSpeed
);
```

This creates chassis speeds (later converted to field-relative in the drive subsystem).

### Odometry: Where Am I?

**File:** [DriveSubsystem.java:44](src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java#L44)

```java
private final SwerveDrivePoseEstimator poseEstimator =
    new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
```

**Odometry** tracks robot position by:
1. Reading wheel encoders (how far each wheel traveled)
2. Reading gyro (which direction robot is facing)
3. Calculating position change (delta)
4. Integrating over time

**Uses:**
- Autonomous path following
- Vision alignment
- Field awareness

**Challenge:** Drift accumulates over time (wheel slip, etc.)

## Utility Classes

### MathHelpers

**File:** [lib/utils/MathHelpers.java](src/main/java/lib/utils/MathHelpers.java)

Provides utility methods like:

```java
public static double getAverage(double a, double b) {
    return (a + b) / 2.0;
}
```

**Used in Superstructure:**
```java
if (m_arm.getPosition() < MathHelpers.getAverage(
    MechanismState.SUBWOOFER.armPositionRadians,
    MechanismState.MID_LOW.armPositionRadians)
) {
    // Arm is closer to SUBWOOFER
}
```

This finds the midpoint between states to determine boundaries!

### TriConsumer

**File:** [lib/functional/TriConsumer.java](src/main/java/lib/functional/TriConsumer.java)

Java provides `Consumer<T>` (one argument) and `BiConsumer<T, U>` (two arguments), but not three!

```java
@FunctionalInterface
public interface TriConsumer<T, U, V> {
    void accept(T t, U u, V v);
}
```

**Use case:** Callbacks that need three parameters.

## Best Practices from This Codebase

### 1. IO Layer Everywhere
- ✅ Every subsystem has an IO interface
- ✅ Hardware and Disabled implementations
- ✅ Testable and replayable

### 2. Centralized Constants
- ✅ All magic numbers in `Constants.java`
- ✅ Organized by subsystem
- ✅ Easy to tune

### 3. State Machine for Coordination
- ✅ Enum defines all valid states
- ✅ Superstructure manages state transitions
- ✅ Clear, predictable behavior

### 4. Command Factories
- ✅ Subsystems provide command factories
- ✅ Superstructure provides high-level commands
- ✅ RobotContainer just binds buttons

### 5. Logging Everything
- ✅ `@AutoLog` for inputs
- ✅ `@AutoLogOutput` for state
- ✅ Every subsystem logs in `periodic()`

### 6. Composition Over Inheritance
- ✅ Commands composed from smaller commands
- ✅ Superstructure coordinates subsystems (not subclasses)
- ✅ IO layers injected (not extended)

### 7. Functional Programming
- ✅ Suppliers for live data
- ✅ Lambdas in command factories
- ✅ Fluent APIs

## Common Mistakes to Avoid

### ❌ Don't Hardcode Values
```java
// Bad
motor.setVoltage(10);

// Good
motor.setVoltage(Constants.IntakeConstants.kIntakeVolts);
```

### ❌ Don't Access Hardware Directly
```java
// Bad (in subsystem)
CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);

// Good
ArmIO io;  // Use IO layer
```

### ❌ Don't Forget Requirements
```java
// Bad
return Commands.run(() -> intake.setVoltage(10));  // No requirement!

// Good
return Commands.run(() -> intake.setVoltage(10), intake);
```

### ❌ Don't Create Objects in Periodic
```java
// Bad
public void periodic() {
    PIDController pid = new PIDController(kP, kI, kD);  // Created every 20ms!
}

// Good
private PIDController pid;  // Created once in constructor
```

### ❌ Don't Block in Periodic
```java
// Bad
public void periodic() {
    Thread.sleep(100);  // NEVER BLOCK!
}

// Good
// Use commands and waitUntil for delays
```

## Where to Go Next

### 1. Explore WPILib Documentation
- https://docs.wpilib.org/
- Command-based programming guide
- PID control tutorial
- Path planning (PathPlanner, Choreo)

### 2. AdvantageKit Resources
- https://github.com/Mechanical-Advantage/AdvantageKit
- Examples from team 6328
- Logging and replay best practices

### 3. Study Other Teams' Code
- Team 6328 (Mechanical Advantage)
- Team 254 (The Cheesy Poofs)
- Team 1678 (Citrus Circuits)

### 4. Practice Challenges

**Challenge 1: Add a Climber**
- Create IO layer for climber
- Add ClimberSubsystem
- Add climb state to MechanismState
- Bind to controller

**Challenge 2: Tune PID**
- Use Shuffleboard to adjust gains live
- Log PID error and output
- Achieve consistent positioning

**Challenge 3: Add Vision**
- Integrate Limelight or PhotonVision
- Create VisionIO layer
- Use vision for odometry corrections

**Challenge 4: Write Autonomous**
- Use PathPlanner for path following
- Create auto command groups
- Add auto chooser to dashboard

### 5. Testing and Validation

**Unit tests:**
```java
@Test
public void testArmMovement() {
    MockArmIO mockIO = new MockArmIO();
    ArmSubsystem arm = new ArmSubsystem(mockIO);

    arm.setVoltage(5.0);
    assertEquals(5.0, mockIO.getLastVoltage());
}
```

**Simulation:**
- Run in SIM mode
- Test logic without hardware
- Verify command sequences

**Log replay:**
- Record a match
- Replay in AdvantageScope
- Debug issues after the fact

## Key Takeaways

1. **Wrapper classes** make hardware intuitive (BeamBreak)
2. **Enhanced controllers** provide better defaults (SamuraiXboxController)
3. **Clutches** enable precision control without mode switching
4. **Swerve drive** is complex but powerful
5. **Odometry** tracks position for autonomous
6. **Best practices** make code maintainable
7. **Keep learning** - FRC software is deep!

## Final Thoughts

You've now learned:
- ✅ The IO layer pattern and AdvantageKit
- ✅ Subsystem architecture (simple and advanced)
- ✅ State machines and coordination
- ✅ Commands and composition
- ✅ Robot lifecycle and timing
- ✅ Custom libraries and utilities

**You're ready to:**
- Understand any code in this project
- Make modifications safely
- Debug issues systematically
- Add new features
- Mentor others!

The best way to learn is to **experiment**:
- Change constants and observe
- Add print statements
- Break things (in SIM mode!)
- Read error messages carefully
- Ask questions

Welcome to FRC programming! 🤖

---

**Navigation:** [← Part 7](07-robot-lifecycle.md) | [Quick Reference →](quick-reference.md)
