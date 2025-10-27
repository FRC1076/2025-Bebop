# Part 2: Understanding the IO Layer Pattern

One of the most important architectural patterns in this codebase is the **IO Layer Pattern**. This pattern is the foundation of AdvantageKit and makes our code testable, debuggable, and replayable.

## The Problem: Hardware and Logic Mixed Together

Traditional robot code might look like this:

```java
public class BadArmSubsystem {
    private CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    public void setVoltage(double volts) {
        motor.setVoltage(volts);  // Hardware access!
    }

    public double getPosition() {
        return encoder.getAbsolutePosition();  // Hardware access!
    }
}
```

**Problems:**
- Can't test without real hardware
- Can't replay logged data to debug
- Can't simulate behavior
- Hard to understand what inputs/outputs exist

## The Solution: IO Layer Pattern

The IO Layer Pattern separates:
1. **What data we need** (the interface)
2. **How we get it** (the implementation)
3. **What we do with it** (the subsystem logic)

### The Three Components

```
┌─────────────────────────────────────────────────────┐
│                   ArmSubsystem                      │
│  (Business logic: PID, state, commands)             │
│  Uses: inputs.positionRadians                       │
│  Calls: io.setVoltage(volts)                       │
└──────────────────┬──────────────────────────────────┘
                   │
         ┌─────────▼─────────┐
         │      ArmIO        │
         │   (Interface)     │
         └─────────┬─────────┘
                   │
         ┌─────────┴─────────┐
         │                   │
   ┌─────▼──────┐    ┌──────▼────────┐
   │ ArmIOHardware│  │ ArmIODisabled │
   │ (Real robot) │  │ (Simulation)  │
   └──────────────┘   └───────────────┘
```

## Deep Dive: The Arm IO Layer

Let's examine the Arm subsystem's IO layer in detail.

### Step 1: The IO Interface

**File:** [ArmIO.java](src/main/java/frc/robot/subsystems/arm/ArmIO.java)

```java
public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double appliedVoltage = 0;
        public double leadMotorCurrentAmps = 0;
        public double followMotorCurrentAmps = 0;
        public double positionRadians = 0;
        public double velocityRadiansPerSecond = 0;
        public double pidTargetRadians = 0;
        public boolean pidRunning = false;
    }

    public abstract void setVoltage(double volts);
    public abstract void updateInputs(ArmIOInputs inputs);
}
```

**What's happening here:**

1. **`@AutoLog` annotation**: AdvantageKit generates logging code automatically for this class
   - Creates `ArmIOInputsAutoLogged` class
   - Handles all NetworkTables publishing
   - Records data for replay

2. **`ArmIOInputs` class**: Contains ALL sensor readings and state
   - All fields are public (unusual in Java, but this is a data container)
   - Starts with default values
   - Gets filled by `updateInputs()`

3. **`setVoltage()` method**: The only OUTPUT (command to hardware)

4. **`updateInputs()` method**: Fills the inputs object with current sensor data

### Step 2: The Hardware Implementation

**File:** [ArmIOHardware.java](src/main/java/frc/robot/subsystems/arm/ArmIOHardware.java)

Let's look at key sections (simplified):

```java
public class ArmIOHardware implements ArmIO {
    private final CANSparkMax leadMotor;
    private final CANSparkMax followMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public ArmIOHardware() {
        // Create motor objects
        leadMotor = new CANSparkMax(
            ArmConstants.kLeadMotorCANId,
            MotorType.kBrushless
        );

        // Configure motors
        leadMotor.setInverted(ArmConstants.kLeadMotorInverted);
        leadMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimitAmps);

        // Create encoder
        absoluteEncoder = new DutyCycleEncoder(
            ArmConstants.kAbsoluteEncoderChannel
        );
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Read from hardware and populate inputs
        inputs.appliedVoltage = leadMotor.getAppliedOutput() *
                                leadMotor.getBusVoltage();
        inputs.leadMotorCurrentAmps = leadMotor.getOutputCurrent();
        inputs.positionRadians = absoluteEncoder.getAbsolutePosition() * 2 * Math.PI
                                - ArmConstants.kAbsoluteEncoderZero
                                + ArmConstants.kAbsoluteEncoderShift;
        // ... more readings
    }

    @Override
    public void setVoltage(double volts) {
        leadMotor.setVoltage(volts);
    }
}
```

**Key points:**

1. **Constructor does hardware setup**: Creates motor objects, sets configurations
2. **`updateInputs()` reads sensors**: Gets current state from hardware
3. **Encoder math**: Converts raw encoder position to radians, applies zero offset
4. **`setVoltage()` sends commands**: Tells motor to run at specified voltage

### Step 3: The Disabled Implementation

**File:** [ArmIODisabled.java](src/main/java/frc/robot/subsystems/arm/ArmIODisabled.java)

```java
public class ArmIODisabled implements ArmIO {
    public ArmIODisabled() {}

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Do nothing - inputs stay at default values
    }

    @Override
    public void setVoltage(double volts) {
        // Do nothing - no hardware to control
    }
}
```

**Why this exists:**
- For simulation/replay mode when no hardware is connected
- Prevents crashes from missing hardware
- Allows testing logic without a robot
- In the future, could add physics simulation here!

### Step 4: Using the IO in the Subsystem

**File:** [ArmSubsystem.java](src/main/java/frc/robot/subsystems/arm/ArmSubsystem.java)

```java
public class ArmSubsystem extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        this.io = io;  // We don't know if it's hardware or disabled!
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);  // Read sensors
        Logger.processInputs("Arm", inputs);  // Log data

        if (runPID) {
            double pidOutput = pidController.calculate(inputs.positionRadians);
            setVoltage(pidOutput);
        }
    }

    public void setVoltage(double volts) {
        // Subsystem logic (software stops)
        if (inputs.positionRadians > ArmConstants.kMaxPositionRadians && volts > 0) {
            volts = 0;
        }
        // ... more logic ...

        io.setVoltage(volts);  // Send to hardware
    }

    public double getPosition() {
        return inputs.positionRadians;  // Read from inputs object
    }
}
```

**Key patterns:**

1. **Constructor takes interface**: `ArmSubsystem(ArmIO io)` - polymorphism!
2. **Store generated inputs object**: `ArmIOInputsAutoLogged`
3. **`periodic()` pattern**:
   - Call `io.updateInputs(inputs)`
   - Call `Logger.processInputs("Arm", inputs)`
   - Do control logic
4. **All sensor reads from `inputs`**: Never call `io.getSomething()`
5. **All outputs through `io`**: `io.setVoltage(volts)`

## How It's Connected in RobotContainer

**File:** [RobotContainer.java:80-107](src/main/java/frc/robot/RobotContainer.java#L80-L107)

```java
if (SystemConstants.currentMode == RobotMode.REAL) {
    m_arm = new ArmSubsystem(new ArmIOHardware());
    m_intake = new IntakeSubsystem(new IntakeIOHardware());
    // ... more subsystems
} else {
    m_arm = new ArmSubsystem(new ArmIODisabled());
    m_intake = new IntakeSubsystem(new IntakeIODisabled());
    // ... more subsystems
}
```

**This is dependency injection!** The subsystem doesn't know if it's running on:
- Real hardware
- Simulation
- Replay from logged data

## The @AutoLog Magic

When you write:

```java
@AutoLog
public static class ArmIOInputs {
    public double positionRadians = 0;
}
```

AdvantageKit's annotation processor generates (at compile time):

```java
public static class ArmIOInputsAutoLogged extends ArmIOInputs {
    public void toLog(LogTable table) {
        table.put("PositionRadians", positionRadians);
    }

    public void fromLog(LogTable table) {
        positionRadians = table.get("PositionRadians", 0.0);
    }
}
```

This means:
- Automatic NetworkTables publishing
- Automatic log file recording
- Automatic replay from logs
- Zero boilerplate code!

## Benefits of This Pattern

### 1. Replay Debugging
Record a match, then replay it later:
```
Match → Log file → Replay in AdvantageScope
                → See all sensor values
                → Find the bug!
```

### 2. Unit Testing
```java
@Test
public void testArmMovement() {
    ArmIO mockIO = new MockArmIO();  // Fake hardware
    ArmSubsystem arm = new ArmSubsystem(mockIO);

    arm.setVoltage(5.0);
    assertEquals(5.0, mockIO.getLastVoltage());
}
```

### 3. Incremental Development
- Write subsystem logic with `IODisabled`
- Test state machines and commands
- Add real hardware implementation later

### 4. Clear Interface Contract
The `ArmIOInputs` class is self-documenting:
- "These are the sensors on the arm"
- "These are their units (radians, amps, etc.)"
- "This is what the subsystem needs to function"

## All IO Layers in This Project

| Subsystem | Interface | Hardware Impl | Disabled Impl |
|-----------|-----------|---------------|---------------|
| Arm | [ArmIO.java](src/main/java/frc/robot/subsystems/arm/ArmIO.java) | [ArmIOHardware.java](src/main/java/frc/robot/subsystems/arm/ArmIOHardware.java) | [ArmIODisabled.java](src/main/java/frc/robot/subsystems/arm/ArmIODisabled.java) |
| Intake | [IntakeIO.java](src/main/java/frc/robot/subsystems/intake/IntakeIO.java) | [IntakeIOHardware.java](src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java) | [IntakeIODisabled.java](src/main/java/frc/robot/subsystems/intake/IntakeIODisabled.java) |
| Index | [IndexIO.java](src/main/java/frc/robot/subsystems/index/IndexIO.java) | [IndexIOHardware.java](src/main/java/frc/robot/subsystems/index/IndexIOHardware.java) | [IndexIODisabled.java](src/main/java/frc/robot/subsystems/index/IndexIODisabled.java) |
| Shooter | [ShooterIO.java](src/main/java/frc/robot/subsystems/shooter/ShooterIO.java) | [ShooterIOHardware.java](src/main/java/frc/robot/subsystems/shooter/ShooterIOHardware.java) | [ShooterIODisabled.java](src/main/java/frc/robot/subsystems/shooter/ShooterIODisabled.java) |
| Gyro | [GyroIO.java](src/main/java/frc/robot/subsystems/drive/GyroIO.java) | [GyroIOPigeon.java](src/main/java/frc/robot/subsystems/drive/GyroIOPigeon.java) / [GyroIONavX.java](src/main/java/frc/robot/subsystems/drive/GyroIONavX.java) | *(not used yet)* |
| Module | [ModuleIO.java](src/main/java/frc/robot/subsystems/drive/ModuleIO.java) | [ModuleIOHardware.java](src/main/java/frc/robot/subsystems/drive/ModuleIOHardware.java) | *(not used yet)* |

## Try It Yourself

### Exercise 1: Find All Intake Inputs
1. Open [IntakeIO.java](src/main/java/frc/robot/subsystems/intake/IntakeIO.java)
2. Find the `IntakeIOInputs` class
3. What sensor readings does the intake have? List them.

### Exercise 2: Trace a Sensor Reading
Starting from [IntakeIOHardware.java](src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java):
1. What CAN ID is the intake motor? (Hint: check the constructor)
2. In `updateInputs()`, what property of the motor gives us current draw?

### Exercise 3: Understand the Disabled Pattern
1. Open [IntakeIODisabled.java](src/main/java/frc/robot/subsystems/intake/IntakeIODisabled.java)
2. What does `updateInputs()` do?
3. What does `setVoltage()` do?
4. Why don't these methods throw errors or log warnings?

### Exercise 4: Find the Interface Usage
1. Open [IntakeSubsystem.java](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java)
2. Find where `io.updateInputs(inputs)` is called
3. Find where `Logger.processInputs()` is called
4. How often do these methods run? (Hint: what method are they in?)

## Common Mistakes to Avoid

### ❌ Don't access hardware directly in subsystem
```java
public class BadArmSubsystem {
    private CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);
    // This bypasses the IO layer!
}
```

### ❌ Don't call getter methods on IO
```java
public void badMethod() {
    double pos = io.getPosition();  // NO! This method doesn't exist
}
```

### ✅ Do read from inputs object
```java
public void goodMethod() {
    double pos = inputs.positionRadians;  // YES! Read from inputs
}
```

### ❌ Don't update inputs yourself
```java
public void badMethod() {
    inputs.positionRadians = 5.0;  // NO! Only IO layer should set these
}
```

### ✅ Do let IO layer update inputs
```java
public void periodic() {
    io.updateInputs(inputs);  // YES! IO layer owns inputs data
}
```

## What's Next?

In **Part 3: Basic Subsystems**, we'll explore:
- How subsystems use the IO layer
- Simple voltage control (Intake, Index)
- Reading Constants
- Creating commands for subsystems

Now that you understand the IO layer foundation, we can build on it!

---

**Navigation:** [← Part 1](01-welcome-and-overview.md) | [Part 3 →](03-subsystems-deep-dive.md)
