# Glossary of FRC and Robot Code Terms

An alphabetical reference of terms used in FRC programming and this codebase.

## A

**Absolute Encoder**
A sensor that measures position and remembers it even when powered off. Used for the arm subsystem to know its angle.

**Autonomous (Auto)**
The first 15 seconds of a match where the robot runs pre-programmed routines without driver input.

**AdvantageKit**
A logging and replay framework created by Team 6328 that enables recording all robot data for post-match debugging.

**@AutoLog**
An annotation that automatically generates logging code for an IO inputs class.

**@AutoLogOutput**
An annotation that automatically logs the return value of a method.

## B

**Beam Break**
A sensor that detects when an object breaks an infrared beam. Used to detect when the robot has a game piece.

**BooleanSupplier**
A functional interface that supplies a boolean value when called. Used for lazy evaluation and live sensor readings.

**Brushless Motor**
A type of motor (like NEO or Falcon 500) that's more efficient than brushed motors. Most FRC robots use brushless motors.

## C

**CAN (Controller Area Network)**
The bus system used to communicate with motor controllers and other devices on the robot. Each device has a unique CAN ID.

**CANSparkMax**
REV Robotics' motor controller for NEO brushless motors.

**ChassisSpeeds**
A WPILib class representing robot velocities (x, y translation and rotation).

**Clutch**
A multiplier (0.0-1.0) applied to drive speeds for precision control. This robot has single (0.6) and double (0.35) clutches.

**Command**
An action or behavior the robot performs. Has lifecycle methods: initialize, execute, isFinished, end.

**Command-Based Programming**
WPILib's framework for organizing robot code using Subsystems, Commands, and Triggers.

**CommandScheduler**
The engine that manages commands, checks requirements, and calls periodic methods. Runs every 20ms.

**Constants**
Configuration values (motor IDs, PID gains, positions, etc.) stored in one centralized file.

## D

**Deadband**
A range of controller input near zero that's treated as zero to prevent drift from stick wear.

**Debounce**
Waiting for a condition to be stable for a duration before acting on it. Prevents false triggers from sensor noise.

**Default Command**
A command that runs on a subsystem whenever no other command is using it.

**Dependency Injection**
Passing dependencies (like IO implementations) into a class rather than creating them inside. Enables testing and flexibility.

**DigitalInput**
A WPILib class for reading on/off digital sensors.

**DoubleSupplier**
A functional interface that supplies a double value when called. Used for reading controller inputs live.

**Drive Station**
The software and computer used by drivers to control the robot and select modes.

## E

**Encoder**
A sensor that measures rotation (position and/or velocity) of a shaft or wheel.

**Enum (Enumeration)**
A Java type that defines a fixed set of constants. Used for MechanismState in this codebase.

## F

**Feedforward**
A control strategy that predicts the voltage needed based on physics, helping PID work better.

**Field-Oriented Drive**
Drive mode where "forward" is always downfield from driver's perspective, regardless of robot rotation.

**Flywheel**
A spinning wheel used to launch game pieces. Must reach precise speeds for accurate shooting.

**FPGA**
Field-Programmable Gate Array - a chip on the RoboRIO that handles precise timing and low-level I/O.

**FRC (FIRST Robotics Competition)**
The high school robotics competition this code is designed for.

## G

**Gradle**
The build system used to compile Java code and deploy to the robot.

**Gyro (Gyroscope)**
A sensor that measures rotation rate and angle. Used for field-oriented drive and odometry.

## I

**IO Layer**
A pattern that separates hardware access (IO) from business logic (subsystem). Enables testing and replay.

**Inversion**
Reversing motor direction in software. Needed when motors are physically mounted backwards.

## J

**Joystick**
Generic term for any game controller used to drive the robot.

## K

**Kinematics**
Math that converts desired robot motion into individual wheel speeds/angles.

## L

**Lambda**
A Java feature for inline functions. Example: `() -> motor.getVoltage()`

**Logging**
Recording data for later analysis. AdvantageKit logs all inputs and outputs automatically.

**LoggedRobot**
AdvantageKit's extended version of TimedRobot that integrates with the logging framework.

## M

**MechanismState**
An enum in this codebase representing all possible configurations of the superstructure (arm angle, roller speeds, etc.).

**Module**
In swerve drive, one wheel assembly with both drive and steering. This robot has 4 modules.

**MutableSuperState**
A class that tracks the current state of the superstructure and whether a note is held.

## N

**NetworkTables**
WPILib's publish-subscribe system for sharing data between robot code, driver station, and dashboards.

**NEO**
REV Robotics' brushless motor, commonly used in FRC.

## O

**Odometry**
Tracking robot position on the field by integrating wheel encoder and gyro data.

**OI (Operator Interface)**
The controllers and button bindings used by drivers.

## P

**Periodic**
Methods that run every 20ms (50 Hz). Every subsystem has a `periodic()` method.

**PID (Proportional-Integral-Derivative)**
A feedback control algorithm that automatically adjusts outputs to reach and maintain a target.

**Pose**
Robot position and rotation on the field (x, y, angle).

**ProfiledPID**
A PID controller that also limits velocity and acceleration for smooth motion.

## R

**RoboRIO**
The main robot controller computer that runs the Java code.

**Replay**
AdvantageKit feature that re-runs logged data through code for debugging.

**Requirement**
A subsystem that a command needs exclusive access to. Prevents conflicting commands.

**Robot-Oriented Drive**
Drive mode where "forward" is the robot's front. Harder to drive when robot rotates.

## S

**Scheduler**
See CommandScheduler.

**Shuffleboard**
WPILib's dashboard application for viewing data and controlling the robot.

**State Machine**
A pattern where a system is always in one of a finite set of states, with defined transitions between them.

**Subsystem**
A robot mechanism (arm, drive, intake, etc.) represented as a class that extends SubsystemBase.

**Superstructure**
A coordinator class that manages multiple subsystems and implements the robot's state machine.

**Supplier**
A functional interface that provides a value when called. Enables lazy evaluation.

**Swerve Drive**
A drive system where each wheel can drive and steer independently, enabling omnidirectional movement.

## T

**Teleop (Teleoperated)**
The driver-controlled portion of the match (after autonomous).

**TimedRobot**
WPILib's base class that provides 20ms periodic methods.

**Trigger**
In WPILib, an object that represents a condition (button press, sensor reading) that can schedule commands.

## U

**USB Stick**
Where log files are written on the real robot. Mounted at `/U/logs`.

## V

**Velocity**
Speed in a direction. Usually measured in meters/second or radians/second.

**Vendor Dependencies (vendordeps)**
Third-party libraries (REV, CTRE, NavX, etc.) used by the robot.

**Voltage Control**
Directly setting motor voltage without feedback. Simple but imprecise.

## W

**WPILib**
The official FRC software library providing motor control, sensors, math utilities, and frameworks.

**WPILOG**
AdvantageKit's binary log file format.

---

**Navigation:** [← Quick Reference](quick-reference.md) | [Back to Start](01-welcome-and-overview.md)
