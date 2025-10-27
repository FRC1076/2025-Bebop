# Part 1: Welcome & Project Overview

Welcome to the 2025 Bebop robot code tutorial! This guide will help you understand how our FRC robot works, from the basics of Java and robotics to the advanced architecture patterns we use.

## What is FRC?

**FIRST Robotics Competition (FRC)** is a high school robotics competition where teams design, build, and program robots to compete in game challenges. Each year brings a new game with different objectives.

In 2025, robots need to:
- Pick up game pieces (notes) from the ground
- Shoot them into goals at various distances and heights
- Navigate and position accurately on the field

## The Command-Based Paradigm

Our robot code uses **WPILib's Command-Based programming**, which organizes robot code into:

1. **Subsystems**: Individual mechanisms (arm, intake, shooter, drive, etc.)
2. **Commands**: Actions the robot performs (move arm up, shoot, drive forward)
3. **Triggers**: Events that start commands (button presses, sensor readings)

Think of it like this:
- **Subsystems** are the "nouns" (the things on your robot)
- **Commands** are the "verbs" (the actions your robot does)
- **Triggers** are the "when" (when to do those actions)

## Key Technologies

### WPILib
The official FRC library that provides:
- Motor control
- Sensor reading
- Controller input
- Command scheduling
- Path following and more

### AdvantageKit
A logging and replay framework that:
- Records ALL sensor data and robot state
- Allows you to replay matches for debugging
- Uses an "IO Layer" pattern (we'll explore this in Part 2)
- Helps you find bugs after the match is over

### Swerve Drive
Our robot uses **swerve drive** - each wheel can:
- Spin forward/backward (drive)
- Rotate independently (turn)

This gives us omnidirectional movement (drive in any direction without turning the robot).

## Project Structure

```
2025-Bebop/
├── src/main/java/
│   ├── frc/robot/               # Main robot code
│   │   ├── Main.java            # Program entry point
│   │   ├── Robot.java           # Robot lifecycle management
│   │   ├── RobotContainer.java  # Subsystems & button bindings
│   │   ├── Constants.java       # All configuration values
│   │   ├── commands/            # Custom commands
│   │   └── subsystems/          # All subsystems
│   │       ├── drive/           # Swerve drive system
│   │       ├── arm/             # Arm subsystem
│   │       ├── intake/          # Intake subsystem
│   │       ├── index/           # Index (conveyor) subsystem
│   │       ├── shooter/         # Shooter subsystem
│   │       └── Superstructure.java  # Coordinates all mechanisms
│   └── lib/                     # Team's reusable libraries
│       ├── hardware/            # Hardware utilities
│       └── utils/               # Helper functions
├── notes/                       # Documentation and diagrams
├── vendordeps/                  # Third-party libraries
└── build.gradle                 # Build configuration
```

## The Robot's State Machine

Our robot operates using a **state machine** - it's always in one state, and transitions between states based on inputs.

![State Machine Diagram](../notes/State Machine Diagram.drawio.png)

### The States

1. **HOME**: Default state, no note, arm down
2. **INTAKE**: Arm down, rollers running to grab note
3. **SUBWOOFER** / **MID_LOW** / **MID_HIGH** / **AMP**: Pre-shooting positions at different angles
4. **SHOOT_XXX**: Actually shooting the note from each position

### How It Works

```
Driver presses left trigger
  → Robot enters INTAKE state (arm down, rollers on)
  → Beam break sensor detects note
  → Automatically transitions to SUBWOOFER state
  → Driver presses Y button to go to AMP position
  → Arm moves up to AMP angle, shooter spins up
  → Driver presses right trigger
  → Robot enters SHOOT_AMP state (index feeds note)
  → Note detected gone by beam break
  → Returns to HOME state
```

This state machine is implemented in [Constants.java:187-292](src/main/java/frc/robot/Constants.java#L187-L292) as the `MechanismState` enum, and managed by [Superstructure.java](src/main/java/frc/robot/subsystems/Superstructure.java).

## Code Flow: From Power-On to Driving

Let's trace what happens when you turn on the robot:

1. **[Main.java:22](src/main/java/frc/robot/Main.java#L22)** - Java starts here
2. **[Robot.java:38](src/main/java/frc/robot/Robot.java#L38)** - Robot constructor runs
   - Sets up AdvantageKit logging
   - Creates `RobotContainer`
3. **[RobotContainer.java:77](src/main/java/frc/robot/RobotContainer.java#L77)** - Container constructor
   - Creates all subsystems (drive, arm, intake, etc.)
   - Creates Superstructure (coordinates subsystems)
   - Binds buttons to commands
4. **[Robot.java:74](src/main/java/frc/robot/Robot.java#L74)** - `robotPeriodic()` runs every 20ms
   - Runs the command scheduler
   - All subsystem `periodic()` methods run
   - Commands execute
   - Logging happens

## Key Files to Know

| File | Purpose | What's Inside |
|------|---------|---------------|
| [Constants.java](src/main/java/frc/robot/Constants.java) | All configuration | Motor IDs, PID values, positions, speeds |
| [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) | Setup & bindings | Creates subsystems, binds buttons |
| [Superstructure.java](src/main/java/frc/robot/subsystems/Superstructure.java) | Mechanism coordinator | State machine, multi-subsystem commands |
| [Robot.java](src/main/java/frc/robot/Robot.java) | Lifecycle manager | Init methods, periodic methods |

## Understanding the Code Comments

You'll see these license headers at the top of many files:

```java
// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project
```

Some files also reference code they're based on:
- WPILib templates (BSD license)
- Team 6328 Mechanical Advantage (AdvantageKit creators)

This is important for open-source compliance and giving credit.

## Try It Yourself

Before moving to Part 2, try these exercises:

### Exercise 1: Find the Drive Speed
1. Open [Constants.java](src/main/java/frc/robot/Constants.java)
2. Find `DriveConstants` class (around line 99)
3. What is the max translation speed? (Hint: it's in meters per second)

### Exercise 2: Find the Intake Motor
1. Open [Constants.java](src/main/java/frc/robot/Constants.java)
2. Find `IntakeConstants` class
3. What CAN ID is the intake motor on?
4. Is it inverted?

### Exercise 3: Trace a Subsystem
1. Look at the file tree under `src/main/java/frc/robot/subsystems/intake/`
2. How many files are there?
3. What do you think each one does? (We'll learn this in Part 2!)

### Exercise 4: Find Button Bindings
1. Open [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)
2. Find the `configureDriverBindings()` method (line 133)
3. What button makes the robot intake? (Hint: look for `.intake()`)

## Common FRC Terms You'll See

- **CAN**: Controller Area Network - how motors communicate
- **PID**: Proportional-Integral-Derivative controller - for precise position/speed control
- **Feedforward**: Predicting how much power is needed
- **Odometry**: Tracking robot position on the field
- **Periodic**: Methods that run every 20ms (50 times per second)
- **Teleop**: Teleoperated - driver control mode
- **Auto**: Autonomous - robot runs pre-programmed routines
- **RoboRIO**: The robot's main computer

## What's Next?

In **Part 2: The IO Layer Pattern**, we'll explore:
- Why we separate hardware code from logic
- How the IO interfaces work
- The difference between Hardware and Disabled implementations
- How AdvantageKit logging works with the IO layer

This is a unique pattern in FRC that makes our code testable and debuggable!

---

**Navigation:** Next: [Part 2: The IO Layer Pattern](02-understanding-io-layers.md)
