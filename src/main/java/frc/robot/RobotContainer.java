// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// The contents of this file are based upon those found in the
// WPILib Command-Based Java Template, which is covered by the BSD license
// found in WPILib-License.md in the root directory of this project

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.index.IndexIOHardware;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import lib.hardware.hid.SamuraiXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    // private final DriveSubsystem m_drive;
    private final ArmSubsystem m_arm;
    private final IndexSubsystem m_index;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final SamuraiXboxController m_driverController =
        new SamuraiXboxController(OIConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if (SystemConstants.currentMode == RobotMode.REAL) {
            m_arm = new ArmSubsystem(new ArmIOHardware());
            m_index = new IndexSubsystem(new IndexIOHardware());
            m_intake = new IntakeSubsystem(new IntakeIOHardware());
            m_shooter = new ShooterSubsystem(new ShooterIOHardware());
        } else {
            // TODO: Replaced with Disabled IO layers
            m_arm = new ArmSubsystem(new ArmIOHardware());
            m_index = new IndexSubsystem(new IndexIOHardware());
            m_intake = new IntakeSubsystem(new IntakeIOHardware());
            m_shooter = new ShooterSubsystem(new ShooterIOHardware());
        }

        // Configure bindings
        configureDriverBindings();
    }

    /** Bind Triggers from the DriverController to Superstructure Commands. */
    private void configureDriverBindings() {
        // final SuperstructureCommandFactory superstructureCommands;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }

    public static Command threadCommand() {
        return Commands.sequence(
                Commands.waitSeconds(20),
                Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 1)),
                Commands.print("Main Thread Priority raised to RT1 at " + Timer.getFPGATimestamp()))
            .ignoringDisable(true);
    }
}
