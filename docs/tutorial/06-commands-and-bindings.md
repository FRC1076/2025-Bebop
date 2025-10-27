# Part 6: Commands & Control

Commands are the heart of WPILib's Command-Based framework. They represent actions the robot can perform and provide a structured way to control robot behavior. Let's explore how commands work and how they're bound to controls.

## What is a Command?

A **command** is an action or behavior that:
- Runs on a schedule managed by the CommandScheduler
- Declares which subsystems it needs
- Has a defined lifecycle (init, execute, end, isFinished)
- Can be composed with other commands

Think of commands as **verbs** - things the robot does.

## Command Lifecycle

Every command has these methods (even if you don't override them):

```java
public class ExampleCommand extends Command {
    // 1. Called once when command is scheduled
    @Override
    public void initialize() {
        // Setup: reset controllers, zero timers, etc.
    }

    // 2. Called repeatedly (every 20ms) while command runs
    @Override
    public void execute() {
        // Main logic: read sensors, compute outputs, control subsystems
    }

    // 3. Should this command stop?
    @Override
    public boolean isFinished() {
        // return true to end command
        // return false to keep running
    }

    // 4. Called once when command ends
    @Override
    public void end(boolean interrupted) {
        // Cleanup: stop motors, release resources
        // interrupted = true if canceled by another command
    }
}
```

### Lifecycle Example: Timed Command

```java
public class RunIntakeForTime extends Command {
    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();

    public RunIntakeForTime(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();  // Start timing
    }

    @Override
    public void execute() {
        intake.setVoltage(10);  // Run at 10V
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3.0);  // Stop after 3 seconds
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();  // Turn off
    }
}
```

## Commands in This Codebase

### Simple Subsystem Commands

Most subsystems provide command factories:

**File:** [IntakeSubsystem.java:32-37](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java#L32-L37)

```java
public Command runVolts(double volts) {
    return Commands.run(
        () -> setVoltage(volts),
        this
    );
}
```

This uses `Commands.run()` - a factory for creating simple commands.

**Breakdown:**
- `Commands.run()` creates a command that runs a lambda repeatedly
- `() -> setVoltage(volts)` is the execute method
- `this` declares the subsystem requirement
- Command never finishes on its own (must be interrupted)

### Stateful Subsystem Commands

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

This demonstrates **command composition**:
- `Commands.sequence()` - runs commands one after another
- `Commands.runOnce()` - runs once, then finishes immediately
- Three setup steps in order

**Why it finishes instantly:** The actual PID runs in `periodic()`, not in a command!

### Custom Command Classes

**File:** [DriveClosedLoopTeleop.java](src/main/java/frc/robot/commands/DriveClosedLoopTeleop.java)

```java
public class DriveClosedLoopTeleop extends Command {
    private final DriveSubsystem m_subsystem;
    private final DoubleSupplier xTransSpeedSupplier;
    private final DoubleSupplier yTransSpeedSupplier;
    private final DoubleSupplier omegaSupplier;
    private final DoubleSupplier translationClutchSupplier;
    private final DoubleSupplier rotationClutchSupplier;

    public DriveClosedLoopTeleop(...suppliers..., DriveSubsystem subsystem) {
        this.xTransSpeedSupplier = xSupplier;
        // ... store all suppliers
        this.m_subsystem = subsystem;

        addRequirements(m_subsystem);  // ← Critical!
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
            scaleSpeed(xTransSpeedSupplier.getAsDouble()) * maxSpeed * clutch,
            scaleSpeed(yTransSpeedSupplier.getAsDouble()) * maxSpeed * clutch,
            omegaSupplier.getAsDouble() * maxRotationSpeed
        );

        m_subsystem.driveCLCO(speeds);
    }
}
```

**Why use suppliers?**
- `DoubleSupplier` is a functional interface: `() -> double`
- Gets values **when execute() runs**, not when command is created
- Allows controller input to change during command execution

**Example:**
```java
new DriveClosedLoopTeleop(
    () -> controller.getLeftY(),  // Reads stick NOW, every execute()
    // not: controller.getLeftY()  // Would read stick ONCE at creation
)
```

## Command Composition

WPILib provides powerful ways to combine commands:

### Sequential Commands

Run one after another:

```java
Commands.sequence(
    armCommands.moveTo(45),
    shooterCommands.spinUp(),
    intakeCommands.feedNote()
)
```

### Parallel Commands

Run at the same time:

```java
Commands.parallel(
    armCommands.moveTo(45),
    shooterCommands.spinUp()
)
```

### Race Commands

Run in parallel, stop all when first finishes:

```java
Commands.race(
    intakeCommands.runForward(),  // Runs forever
    Commands.waitSeconds(3)        // Finishes after 3s
)  // Both end after 3 seconds
```

### Deadline Commands

Run in parallel, stop all when deadline finishes:

```java
Commands.deadline(
    Commands.waitSeconds(3),     // Deadline - determines when to stop
    intakeCommands.runForward(), // Runs along
    shooterCommands.spinSlow()   // Runs along
)  // All end when deadline ends
```

### Example from Superstructure

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

**Composition breakdown:**
1. `Commands.sequence` - do things in order
2. `applyStateArmFirst` - returns a command
3. `Commands.waitUntil` - waits for condition
4. `applyStateAllParallel` - returns another command

**Commands composing commands!**

## Button Bindings

Triggers (button presses, sensor readings, etc.) can schedule commands.

### Trigger Types

**File:** [RobotContainer.java:139-160](src/main/java/frc/robot/RobotContainer.java#L139-L160)

```java
// Button triggers
m_driverController.leftTrigger()   // Controller button
m_driverController.a()              // Controller button
m_driverController.povUp()          // D-pad direction
```

### Binding Methods

#### `whileTrue(command)`
Command runs while button is held, cancels when released:

```java
m_driverController.leftTrigger()
    .whileTrue(superstructureCommands.intake());
```

#### `onTrue(command)`
Command starts when button is pressed, runs until finished (even if released):

```java
m_driverController.a()
    .onTrue(superstructureCommands.subwoofer());
```

#### `onFalse(command)`
Command starts when button is released:

```java
m_driverController.povUp()
    .whileTrue(superstructureCommands.armUpManual())
    .onFalse(superstructureCommands.detectMechanismState());
```

This is **chaining** - when you release POV up, detect state!

#### `toggleOnTrue(command)`
Pressing button toggles command on/off:

```java
controller.x()
    .toggleOnTrue(intakeCommands.runContinuous());
// Press once: starts
// Press again: stops
```

### Trigger Modifiers

#### `and()`
Both conditions must be true:

```java
controller.a().and(controller.b())
    .onTrue(specialCommand);
```

#### `or()`
Either condition being true:

```java
controller.leftBumper().or(controller.rightBumper())
    .whileTrue(slowModeCommand);
```

#### `negate()`
Inverts the condition:

```java
controller.a().negate()
    .whileTrue(defaultCommand);
// Runs when A is NOT pressed
```

#### `debounce(seconds)`
Condition must be stable for duration:

```java
new Trigger(beamBreak::isBroken)
    .debounce(0.1)
    .onTrue(Commands.print("Note detected!"));
// Avoids false positives from bouncing
```

### Complex Binding Example

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
- While right bumper is held AND left bumper is NOT held: slow mode (0.6x speed)
- When right bumper is released AND left bumper is NOT held: return to full speed

This prevents conflicts between single clutch and double clutch!

## The Command Scheduler

The scheduler is the engine that runs commands. It's called every 20ms in `Robot.periodic()`.

**File:** [Robot.java:74-80](src/main/java/frc/robot/Robot.java#L74-L80)

```java
@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();
}
```

### What the Scheduler Does

Every 20ms:

1. **Poll all triggers** - check if buttons/conditions have changed
2. **Schedule new commands** - from trigger bindings
3. **Check requirements** - cancel conflicting commands
4. **Run commands**:
   - Call `execute()` on running commands
   - Check `isFinished()`
   - Call `end()` if finished
5. **Run subsystem `periodic()`** methods
6. **Update logging**

### Subsystem Requirements

**Why requirements matter:**

```java
Command cmd1 = intake.runVolts(10);   // Requires IntakeSubsystem
Command cmd2 = intake.runVolts(-10);  // Requires IntakeSubsystem

cmd1.schedule();  // Starts running
cmd2.schedule();  // cmd1 is CANCELLED, cmd2 starts

// Without requirements, both would run → motors fight!
```

**How to declare:**

```java
public MyCommand(IntakeSubsystem intake) {
    addRequirements(intake);  // In constructor
}
```

Or with functional commands:

```java
Commands.run(() -> intake.setVoltage(5), intake);
                                        //  ↑ requirement
```

## Functional Command Factories

WPILib provides shortcuts for common patterns:

### `Commands.run(runnable, requirements)`
Runs repeatedly, never finishes:

```java
Commands.run(() -> intake.setVoltage(10), intake)
```

### `Commands.runOnce(runnable, requirements)`
Runs once, finishes immediately:

```java
Commands.runOnce(() -> arm.setPidTarget(45), arm)
```

### `Commands.waitSeconds(seconds)`
Waits for time:

```java
Commands.waitSeconds(2.0)
```

### `Commands.waitUntil(condition)`
Waits for condition:

```java
Commands.waitUntil(() -> arm.atTarget())
```

### `Commands.print(message)`
Prints to console (useful for debugging):

```java
Commands.print("Intake finished!")
```

### `Commands.none()`
Does nothing (useful for default commands):

```java
Commands.none()
```

## Default Commands

Subsystems can have a **default command** that runs when no other command is using them.

**File:** [RobotContainer.java:117-126](src/main/java/frc/robot/RobotContainer.java#L117-L126)

```java
m_drive.setDefaultCommand(
    new DriveClosedLoopTeleop(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> translationalClutch,
        () -> rotationalClutch,
        m_drive
    )
);
```

**This means:**
- Whenever no other command needs `m_drive`
- The default teleop drive command runs
- Allows driver to always control the robot unless an auto command takes over

## Try It Yourself

### Exercise 1: Understand Button Binding Types
For each binding, determine what happens:

```java
// 1.
controller.a().onTrue(command);

// 2.
controller.b().whileTrue(command);

// 3.
controller.x().toggleOnTrue(command);
```

Questions:
- What happens when you press the button?
- What happens when you release it?
- Can you interrupt it by pressing another button?

### Exercise 2: Trace a Command Sequence
Given this command:
```java
Commands.sequence(
    Commands.print("Starting"),
    Commands.waitSeconds(1),
    arm.moveTo(45),
    Commands.waitUntil(() -> arm.atTarget()),
    Commands.print("Done")
)
```

1. What prints first?
2. When does the arm start moving?
3. When does "Done" print?
4. Can this be interrupted?

### Exercise 3: Identify Requirements
For each command, identify required subsystems:

```java
// 1.
Commands.run(() -> intake.setVoltage(10), intake)

// 2.
Commands.parallel(
    arm.moveTo(45),
    shooter.spinUp()
)

// 3.
Commands.sequence(
    intake.runForward(),
    Commands.waitSeconds(2),
    intake.stop()
)
```

### Exercise 4: Design a Button Binding
You want a button that:
- Moves arm to 45° when pressed
- Spins shooter to 500 rad/s when arm reaches target
- Runs intake to feed note
- Returns to home when finished

Write the command composition and binding.

### Exercise 5: Debug a Trigger
This binding doesn't work as expected:

```java
controller.a().and(controller.b())
    .whileTrue(specialCommand);
```

Problem: `specialCommand` never runs, even when both buttons are pressed.

What's the issue? (Hint: think about `whileTrue` vs `onTrue`)

## Common Patterns

### Pattern 1: State Setup Command
```java
public Command setState() {
    return Commands.sequence(
        Commands.runOnce(() -> controller.reset()),
        Commands.runOnce(() -> setTarget(goal)),
        Commands.runOnce(() -> enable())
    );
}
```
Finishes instantly, state changes happen in `periodic()`.

### Pattern 2: Run Until Condition
```java
public Command intakeUntilNote() {
    return Commands.sequence(
        intake.runForward(),
        Commands.waitUntil(() -> hasNote())
    );
}
```

### Pattern 3: Timeout Wrapper
```java
myCommand.withTimeout(5.0)  // Cancels after 5 seconds if not done
```

### Pattern 4: Conditional Command
```java
Commands.either(
    planA,  // If condition is true
    planB,  // If condition is false
    () -> someCondition()
)
```

## Key Takeaways

1. **Commands have a lifecycle** - initialize, execute, isFinished, end
2. **Functional factories** - `Commands.run()`, `Commands.sequence()`, etc.
3. **Composition is powerful** - combine simple commands into complex behaviors
4. **Triggers schedule commands** - button presses, sensor readings, timers
5. **Requirements prevent conflicts** - only one command per subsystem
6. **Suppliers provide live data** - read values when execute() runs
7. **Default commands** - run when subsystem is idle

## What's Next?

In **Part 7: Robot Lifecycle**, we'll explore:
- How the robot boots up
- Main, Robot, and RobotContainer classes
- Different robot modes (disabled, auto, teleop)
- Periodic methods and timing
- AdvantageKit logging setup

Let's see how everything initializes and runs!

---

**Navigation:** [← Part 5](05-superstructure-architecture.md) | [Part 7 →](07-robot-lifecycle.md)
