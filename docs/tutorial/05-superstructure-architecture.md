# Part 5: The Superstructure Architecture

The **Superstructure** is the conductor of our robot's orchestra. It coordinates all the mechanism subsystems (arm, intake, index, shooter) to work together as a cohesive system. This is one of the most important architectural patterns in competitive FRC code.

## Why Do We Need a Superstructure?

### The Problem: Subsystem Isolation

Without a Superstructure, each subsystem only knows about itself:

```java
// ❌ This creates problems:
arm.moveToAngle(45);
shooter.spinUp(500);
intake.runForward();
// Who coordinates them? When does each start? What if they conflict?
```

### The Solution: Centralized Coordination

The Superstructure knows about ALL mechanisms and can:
- Coordinate timing (arm moves THEN shooter spins)
- Manage state (what's the robot trying to do?)
- Share sensor data (beam break sensor affects multiple subsystems)
- Create complex multi-subsystem commands

```java
// ✅ Superstructure handles complexity:
superstructure.intake();  // Coordinates arm, intake, index, AND sensors!
```

## The State Machine Pattern

Remember the state diagram from Part 1? The Superstructure implements it!

![State Machine](../notes/State Machine Diagram.drawio.png)

### States as an Enum

**File:** [Constants.java:187-292](src/main/java/frc/robot/Constants.java#L187-L292)

```java
public enum MechanismState {
    HOME(
        0,              // intakeVolts
        0,              // indexVolts
        -0.6457718,     // armPositionRadians
        0,              // shooterLeftSpeedRadPerSec
        0               // shooterRightSpeedRadPerSec
    ),

    INTAKE(
        10,             // intakeVolts - run to grab note
        4,              // indexVolts - pull into robot
        -0.6457718,     // armPositionRadians - arm down
        0,              // shooterLeftSpeedRadPerSec - off
        0               // shooterRightSpeedRadPerSec - off
    ),

    SUBWOOFER(
        0,              // intakeVolts - off
        0,              // indexVolts - holding note
        -0.4014257,     // armPositionRadians - low angle
        -471,           // shooterLeftSpeedRadPerSec - spinning up
        576             // shooterRightSpeedRadPerSec
    ),

    // ... MID_LOW, MID_HIGH, AMP, and all SHOOT_XXX states
}
```

**Why this is brilliant:**

1. **Compile-time safety**: Can't typo a state name
2. **Self-documenting**: Each state shows exactly what every subsystem does
3. **Easy to tune**: Change one number, affects whole robot behavior
4. **Type-safe**: Java won't let you pass invalid states

### State Fields

```java
public final double intakeVolts;
public final double indexVolts;
public final double armPositionRadians;
public final double shooterLeftSpeedRadPerSec;
public final double shooterRightSpeedRadPerSec;
```

Each state is a **snapshot of all mechanism positions/speeds**.

## Superstructure Architecture

**File:** [Superstructure.java](src/main/java/frc/robot/subsystems/Superstructure.java)

### The Class Structure

```java
public class Superstructure {
    // Subsystems
    private final ArmSubsystem m_arm;
    private final IndexSubsystem m_index;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;

    // State tracking
    private final MutableSuperState superState;

    // Command factory
    public final SuperstructureCommandFactory commandBuilder;

    public Superstructure(
        ArmSubsystem arm,
        IndexSubsystem index,
        IntakeSubsystem intake,
        ShooterSubsystem shooter,
        BooleanSupplier hasNoteSupplier
    ) {
        this.m_arm = arm;
        this.m_index = index;
        this.m_intake = intake;
        this.m_shooter = shooter;

        this.superState = new MutableSuperState(hasNoteSupplier);
        this.commandBuilder = new SuperstructureCommandFactory(this);
    }
}
```

**Key components:**

1. **Owns all mechanism subsystems** - can command any of them
2. **Tracks state** - knows what the robot is doing
3. **Has a command factory** - creates multi-subsystem commands

### The MutableSuperState

**File:** [Superstructure.java:34-65](src/main/java/frc/robot/subsystems/Superstructure.java#L34-L65)

```java
public static class MutableSuperState {
    protected MechanismState mechanismState;
    protected BooleanSupplier hasNote;

    public void setMechanismState(MechanismState state) {
        this.mechanismState = state;
    }

    public MechanismState getMechanismState() {
        return this.mechanismState;
    }

    @AutoLogOutput(key="SuperState/MechanismState")
    public String getMechanismStateString() {
        return this.mechanismState.toString();
    }

    @AutoLogOutput(key="SuperState/HasNote")
    public boolean hasNote() {
        return this.hasNote.getAsBoolean();
    }
}
```

**Why mutable?**
- State changes throughout match (HOME → INTAKE → SUBWOOFER → etc.)
- Needs to be updated by commands
- Logged automatically via `@AutoLogOutput`

**The `hasNote` supplier:**
```java
// In RobotContainer
m_beamBreak = new BeamBreak(OIConstants.kBeamBreakPin);

m_superstructure = new Superstructure(
    m_arm,
    m_index,
    m_intake,
    m_shooter,
    m_beamBreak.beamBrokenSupplier()  // ← Function that checks sensor
);
```

This allows the Superstructure to react to sensor readings!

## Applying States: Three Strategies

The Superstructure has three methods for applying states, each for different scenarios.

### Strategy 1: Arm First, Then Others

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

**When used:** INTAKE state - arm must be down before turning on rollers!

**Flow:**
1. Move arm to position
2. Wait for arm to reach position
3. Then start all rollers

**Why:** Prevents trying to intake with arm in the wrong position.

### Strategy 2: All Parallel

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

**When used:** Pre-shooting positions (SUBWOOFER, MID_HIGH, etc.)

**Flow:**
- Everything starts at once
- Arm moves while shooter spins up
- Faster than sequential!

**Why:** Get ready to shoot as quickly as possible.

### Strategy 3: No Arm Movement

**File:** [Superstructure.java:185-193](src/main/java/frc/robot/subsystems/Superstructure.java#L185-L193)

```java
private Command applyStateNoArmMove(MechanismState state) {
    superState.setMechanismState(state);

    return Commands.parallel(
        m_intake.runVolts(state.intakeVolts),
        m_index.runVolts(state.indexVolts),
        m_shooter.startPid(state.shooterLeftSpeedRadPerSec, state.shooterRightSpeedRadPerSec)
    );
}
```

**When used:** SHOOT states - arm already in position!

**Flow:**
- Arm stays where it is
- Only rollers change

**Why:** Don't waste time repositioning arm when shooting.

## The Command Factory Pattern

**File:** [Superstructure.java:198-312](src/main/java/frc/robot/subsystems/Superstructure.java#L198-L312)

The `SuperstructureCommandFactory` is an inner class that creates all the robot's high-level commands.

### Why a Factory?

```java
// ❌ Without factory:
new SequentialCommandGroup(
    new ParallelCommandGroup(
        arm.startPid(0.5),
        shooter.startPid(500, 600)
    ),
    new WaitCommand(0.2),
    intake.runVolts(10)
);  // Long, ugly, hard to read!

// ✅ With factory:
superstructureCommands.intake();  // Clear and simple!
```

**Benefits:**
- Encapsulates complexity
- Reusable across button bindings
- Self-documenting
- Easy to test

### The Factory Constructor

```java
public class SuperstructureCommandFactory {
    private final Superstructure superstructure;
    private final Map<MechanismState, MechanismState> scoringStates = new HashMap<>();

    private SuperstructureCommandFactory(Superstructure superstructure) {
        this.superstructure = superstructure;

        scoringStates.put(MechanismState.SUBWOOFER, MechanismState.SHOOT_SUBWOOFER);
        scoringStates.put(MechanismState.MID_LOW, MechanismState.SHOOT_MID_LOW);
        scoringStates.put(MechanismState.MID_HIGH, MechanismState.SHOOT_MID_HIGH);
        scoringStates.put(MechanismState.AMP, MechanismState.SHOOT_AMP);
    }
}
```

**The `scoringStates` map:**
- Pre-shooting state → actual shooting state
- Used by `shoot()` command to know which SHOOT state to use
- Smart mapping instead of if/else chains!

## Key Commands Explained

### The Intake Command

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

**Step-by-step:**

1. **Apply INTAKE state**
   - Arm moves down (if needed)
   - Wait for arm
   - Turn on rollers

2. **Wait for note**
   - `Commands.waitUntil()` blocks until condition is true
   - Checks beam break sensor via `hasNote()`

3. **Go to SUBWOOFER**
   - Arm moves to shooting angle
   - Rollers stop (hold note)
   - Shooter spins up
   - Ready to fire!

**This is automatic sequencing!** Driver just holds trigger, robot does the rest.

### The Shoot Command

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

**Step-by-step:**

1. **Transition to SHOOT state**
   - Look up current state in `scoringStates` map
   - If SUBWOOFER → SHOOT_SUBWOOFER
   - If AMP → SHOOT_AMP
   - Default to SHOOT_MID_HIGH if somehow not in a known state
   - Turn on index roller (feeds note)

2. **Wait for note to leave**
   - `hasNote.negate()` = wait for note to be gone
   - `.debounce(0.2)` = must be gone for 0.2 seconds
   - Prevents false positives from note bouncing

3. **Return home**
   - Arm down
   - Everything off
   - Ready for next cycle

### Manual Control Commands

**File:** [Superstructure.java:298-311](src/main/java/frc/robot/subsystems/Superstructure.java#L298-L311)

```java
public Command armUpManual() {
    return Commands.run(
        () -> m_arm.setVoltage(SuperstructureConstants.kArmManualControlVoltage),
        m_arm
    );
}

public Command armDownManual() {
    return Commands.run(
        () -> m_arm.setVoltage(SuperstructureConstants.kArmManualControlVoltage * -1),
        m_arm
    );
}
```

**Used for:**
- Testing individual subsystems
- Recovery from errors
- Fine-tuning positions

**Bound to D-pad:** [RobotContainer.java:163-170](src/main/java/frc/robot/RobotContainer.java#L163-L170)

```java
m_driverController.povUp()
    .whileTrue(superstructureCommands.armUpManual())
    .onFalse(superstructureCommands.detectMechanismState());

m_driverController.povDown()
    .whileTrue(superstructureCommands.armDownManual())
    .onFalse(superstructureCommands.detectMechanismState());
```

**Note the `.onFalse()`** - when you release the D-pad, the robot detects what state it should be in based on arm position!

### The Detect State Method

**File:** [Superstructure.java:118-139](src/main/java/frc/robot/subsystems/Superstructure.java#L118-L139)

```java
public void detectState() {
    if (!superState.hasNote()) {
        superState.setMechanismState(MechanismState.HOME);
    } else if (m_arm.getPosition() < MathHelpers.getAverage(
        MechanismState.SUBWOOFER.armPositionRadians,
        MechanismState.MID_LOW.armPositionRadians)
    ) {
        superState.setMechanismState(MechanismState.SUBWOOFER);
    } else if (m_arm.getPosition() < MathHelpers.getAverage(
        MechanismState.MID_LOW.armPositionRadians,
        MechanismState.MID_HIGH.armPositionRadians)
    ) {
        superState.setMechanismState(MechanismState.MID_LOW);
    } else if (m_arm.getPosition() < MathHelpers.getAverage(
        MechanismState.MID_HIGH.armPositionRadians,
        MechanismState.AMP.armPositionRadians)
    ) {
        superState.setMechanismState(MechanismState.MID_HIGH);
    } else {
        superState.setMechanismState(MechanismState.AMP);
    }
}
```

**Logic:**
- If no note → HOME
- Otherwise, find which state's arm angle is closest
- Uses midpoints between states as boundaries

**Why it's needed:**
After manual control, the robot needs to "snap" to the nearest valid state.

## How It All Connects

### In RobotContainer

**File:** [RobotContainer.java:109-115](src/main/java/frc/robot/RobotContainer.java#L109-L115)

```java
m_superstructure = new Superstructure(
    m_arm,
    m_index,
    m_intake,
    m_shooter,
    m_beamBreak.beamBrokenSupplier()
);
```

### Getting Command Factory

**File:** [RobotContainer.java:134](src/main/java/frc/robot/RobotContainer.java#L134)

```java
final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandbuilder();
```

### Binding to Buttons

**File:** [RobotContainer.java:139-160](src/main/java/frc/robot/RobotContainer.java#L139-L160)

```java
m_driverController.leftTrigger()
    .whileTrue(superstructureCommands.intake());

m_driverController.rightTrigger()
    .whileTrue(superstructureCommands.shoot());

m_driverController.a()
    .onTrue(superstructureCommands.subwoofer());

m_driverController.b()
    .onTrue(superstructureCommands.midLow());

m_driverController.x()
    .onTrue(superstructureCommands.midHigh());

m_driverController.y()
    .onTrue(superstructureCommands.amp());
```

**The complete driver experience:**
1. Hold left trigger → robot intakes automatically
2. Press A/B/X/Y → robot positions for that shot
3. Hold right trigger → robot shoots
4. Release trigger → robot returns home

All the complexity is hidden!

## Try It Yourself

### Exercise 1: Trace an Intake Cycle
Starting from [Superstructure.java:219](src/main/java/frc/robot/subsystems/Superstructure.java#L219):
1. What method applies the INTAKE state?
2. Does the arm move before or after the rollers start?
3. What sensor determines when intaking is done?
4. What state does the robot go to after intaking?

### Exercise 2: Understanding State Mapping
1. Open [Superstructure.java:205-208](src/main/java/frc/robot/subsystems/Superstructure.java#L205-L208)
2. What does the `scoringStates` map do?
3. If you're in MID_HIGH and press the shoot trigger, what state do you go to?
4. Why use a map instead of if/else statements?

### Exercise 3: Decode the Shoot Sequence
1. In the `shoot()` command, what happens first?
2. Why does it use `applyStateNoArmMove()` instead of `applyStateAllParallel()`?
3. What does `.negate()` do on the trigger?
4. Why debounce for 0.2 seconds?

### Exercise 4: Compare State Application Strategies
Create a table comparing the three strategies:
| Strategy | When Used | Subsystems Affected | Sequential or Parallel? |
|----------|-----------|---------------------|-------------------------|
| applyStateArmFirst | ? | ? | ? |
| applyStateAllParallel | ? | ? | ? |
| applyStateNoArmMove | ? | ? | ? |

### Exercise 5: Add a New State
On paper, design a new state called CLIMB for end-game:
1. What should each subsystem do? (arm angle, roller speeds, etc.)
2. Would you need a CLIMB_ACTIVE state too (like SHOOT states)?
3. What button would activate it?
4. What command would you create in the factory?

## Design Patterns in Superstructure

### Pattern 1: Coordinator, Not Controller
```java
// Superstructure doesn't control hardware directly
m_arm.startPid(angle);  // Asks subsystem to do something

// Not:
armMotor.setVoltage(5);  // Superstructure shouldn't know about motors!
```

### Pattern 2: State as Data
```java
// States are just data structures
MechanismState.SUBWOOFER.armPositionRadians;

// Not methods or behaviors
```

### Pattern 3: Factory for Complexity
```java
// Public interface is simple
superstructureCommands.intake();

// Implementation is complex
Commands.sequence(
    applyStateArmFirst(MechanismState.INTAKE),
    Commands.waitUntil(hasNote),
    applyStateAllParallel(MechanismState.SUBWOOFER)
);
```

### Pattern 4: Sensor Injection
```java
// Superstructure doesn't own sensors
new Superstructure(..., hasNoteSupplier);

// RobotContainer owns sensors
m_beamBreak.beamBrokenSupplier()
```

## Key Takeaways

1. **Superstructure coordinates subsystems** - they don't coordinate themselves
2. **State machine pattern** - enum defines all robot configurations
3. **Three application strategies** - sequential, parallel, or partial
4. **Command factory** - encapsulates complex command composition
5. **Sensor integration** - beam break affects state transitions
6. **Automatic sequences** - driver presses one button, robot does many things
7. **Manual overrides** - always provide a way to control individual subsystems

## What's Next?

In **Part 6: Commands & Control**, we'll explore:
- WPILib command framework in detail
- Button binding strategies
- Command composition (sequence, parallel, race, etc.)
- Triggers and conditions
- How the command scheduler works

Now that you understand what commands DO, let's learn how they WORK!

---

**Navigation:** [← Part 4](04-advanced-subsystems.md) | [Part 6 →](06-commands-and-bindings.md)
