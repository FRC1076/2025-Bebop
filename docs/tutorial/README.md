# 2025 Bebop Robot Code Tutorial

Welcome to the comprehensive onboarding tutorial for FRC Team 1076's 2025 Bebop robot! This tutorial series will take you from FRC newcomer to confident robot programmer.

## Who Is This For?

This tutorial is designed for:
- Students and mentors new to FRC programming
- Programmers who know Java but not FRC
- Team members who want to understand the codebase deeply
- Anyone transitioning from other programming paradigms to Command-Based

## What You'll Learn

By the end of this series, you'll understand:
- ✅ WPILib's Command-Based programming framework
- ✅ The IO Layer pattern and AdvantageKit logging
- ✅ Subsystem architecture (from simple to advanced)
- ✅ State machines and multi-subsystem coordination
- ✅ PID control and feedforward
- ✅ Commands, triggers, and button bindings
- ✅ Robot lifecycle and timing
- ✅ Custom libraries and best practices

## Prerequisites

**Required:**
- Basic Java knowledge (classes, methods, variables, inheritance)
- Ability to read code and follow logic
- Curiosity and willingness to experiment

**Helpful but not required:**
- Understanding of interfaces and polymorphism
- Functional programming concepts (lambdas, suppliers)
- Basic control theory (PID)
- FRC game knowledge

## Tutorial Structure

### Core Tutorial (8 Parts)

Follow these in order for the best learning experience:

#### [Part 1: Welcome & Project Overview](01-welcome-and-overview.md) ⏱️ 15 min
Get oriented with FRC, Command-Based programming, and the project structure.
- What is FRC and WPILib?
- Project file organization
- The robot's state machine
- Key files to know

#### [Part 2: Understanding the IO Layer Pattern](02-understanding-io-layers.md) ⏱️ 25 min
Learn the architectural foundation that makes this code testable and replayable.
- Why separate hardware from logic?
- IO interfaces, Hardware implementations, Disabled implementations
- AdvantageKit's @AutoLog magic
- Benefits of the pattern

#### [Part 3: Basic Subsystems Deep Dive](03-subsystems-deep-dive.md) ⏱️ 30 min
Explore simple subsystems with voltage control.
- Anatomy of a subsystem
- Intake and Index subsystems
- Connecting to Constants
- Command factories

#### [Part 4: Advanced Subsystems](04-advanced-subsystems.md) ⏱️ 40 min
Master closed-loop control with PID and feedforward.
- PID fundamentals
- Position control (Arm)
- Velocity control (Shooter)
- Feedforward explained

#### [Part 5: The Superstructure Architecture](05-superstructure-architecture.md) ⏱️ 45 min
Understand how all subsystems work together.
- Why we need a Superstructure
- State machine implementation
- Three application strategies
- The command factory pattern

#### [Part 6: Commands & Control](06-commands-and-bindings.md) ⏱️ 35 min
Deep dive into WPILib's command framework.
- Command lifecycle
- Command composition (sequence, parallel, race)
- Button bindings and triggers
- The command scheduler

#### [Part 7: Robot Lifecycle](07-robot-lifecycle.md) ⏱️ 30 min
Trace robot execution from power-on to match play.
- Main → Robot → RobotContainer
- Robot modes (disabled, auto, teleop)
- The periodic heartbeat
- AdvantageKit logging setup

#### [Part 8: Libraries & Advanced Topics](08-libraries-and-advanced.md) ⏱️ 40 min
Explore custom utilities and advanced features.
- BeamBreak sensor wrapper
- SamuraiXboxController enhancements
- Drive clutches
- Swerve drive and odometry basics
- Best practices

**Total estimated time: ~4 hours**

### Reference Materials

Quick lookups for when you need specific information:

#### [Quick Reference Sheet](quick-reference.md)
Fast lookup for common patterns, file locations, and code snippets.
- File locations
- Motor CAN IDs
- Common constants
- Code patterns
- Controller bindings
- Debugging tips

#### [Glossary](glossary.md)
Definitions of FRC and programming terms used throughout the codebase.
- Alphabetical reference
- FRC-specific terms
- Programming concepts
- Hardware terminology

## How to Use This Tutorial

### For Self-Study

1. **Read in order** - Each part builds on previous knowledge
2. **Do the exercises** - Every part has "Try It Yourself" sections
3. **Experiment** - Change code in SIM mode and observe results
4. **Take notes** - Write down questions and look them up
5. **Review** - Come back to earlier parts as you learn more

### For Team Training

1. **Week 1-2**: Parts 1-3 (Foundations and simple subsystems)
2. **Week 3-4**: Parts 4-5 (Advanced control and coordination)
3. **Week 5-6**: Parts 6-7 (Commands and lifecycle)
4. **Week 7-8**: Part 8 and hands-on projects

### For Quick Reference

- Use the **Quick Reference** for common patterns
- Use the **Glossary** when you encounter unfamiliar terms
- Search within files for specific topics (all tutorials are markdown)

## Learning Tips

### Active Learning
- **Don't just read** - Open the actual files and follow along
- **Use your IDE's "Go to Definition"** to jump between files
- **Add print statements** to see execution flow
- **Break things** (in SIM mode!) to understand error messages

### When You Get Stuck
1. Re-read the relevant section slowly
2. Look at the actual code file referenced
3. Check the Quick Reference for examples
4. Try explaining the concept to someone else
5. Ask for help (mentors, team leads, Chief Delphi)

### Practice Projects

After completing the tutorial, try these:

**Beginner:**
- Add a new simple subsystem (like a climber)
- Modify an existing state's parameters
- Add a new button binding

**Intermediate:**
- Create a new MechanismState for climbing
- Tune PID controllers for better performance
- Add logging for a new measurement

**Advanced:**
- Implement vision tracking with PhotonVision
- Write autonomous routines with PathPlanner
- Add unit tests for subsystems

## Additional Resources

### Official Documentation
- [WPILib Docs](https://docs.wpilib.org/) - Official FRC programming documentation
- [AdvantageKit GitHub](https://github.com/Mechanical-Advantage/AdvantageKit) - Logging framework docs and examples

### Community Resources
- [Chief Delphi](https://www.chiefdelphi.com/) - FRC community forum
- [FRC Discord](https://discord.gg/frc) - Real-time chat and help
- [Team 6328 Code](https://github.com/Mechanical-Advantage/RobotCode2024) - Examples from AdvantageKit creators

### Example Code
- [Team 254](https://github.com/Team254) - The Cheesy Poofs (competitive veteran team)
- [Team 1678](https://github.com/frc1678) - Citrus Circuits (excellent architecture)
- [Team 6328](https://github.com/Mechanical-Advantage) - Mechanical Advantage (AdvantageKit creators)

## Getting Help

### Within This Tutorial
- Each part links to actual code files with line numbers
- Exercises reinforce concepts with hands-on practice
- Glossary defines all technical terms

### From Your Team
- Ask mentors and experienced students
- Pair program with teammates
- Do code reviews together

### From the Community
- **Chief Delphi** for general FRC programming questions
- **FRC Discord** for real-time help
- **GitHub Issues** on WPILib for bugs/features

## Contributing to This Tutorial

Found a typo? Have a suggestion? Want to add examples?

1. Open an issue or submit a pull request
2. Suggest improvements to team programming leads
3. Add your own tips and tricks you've learned

## Acknowledgments

This tutorial was created for FRC Team 1076 PiHi Samurai based on:
- WPILib Command-Based programming framework
- AdvantageKit logging framework by Team 6328
- Best practices from top FRC teams
- Real code from the 2025 Bebop robot

Special thanks to all contributors, mentors, and students who helped make this possible!

---

## Ready to Start?

**Begin your journey:** [Part 1: Welcome & Project Overview →](01-welcome-and-overview.md)

Good luck, and welcome to FRC programming! 🤖⚙️🔧
