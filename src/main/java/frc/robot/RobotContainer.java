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
import frc.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;
import frc.robot.Constants.OIConstants.SecondaryControllerStates;
import frc.robot.commands.Autos;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.subsystems.arm.ArmIODisabled;
import frc.robot.subsystems.arm.ArmIOHardware;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.drive.DriveSubsystem.DriveCommandFactory;
import frc.robot.subsystems.index.IndexIODisabled;
import frc.robot.subsystems.index.IndexIOHardware;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.intake.IntakeIODisabled;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIODisabled;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import lib.hardware.BeamBreak;
import lib.hardware.hid.SamuraiXboxController;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final DriveSubsystem m_drive;
    private final ArmSubsystem m_arm;
    private final IndexSubsystem m_index;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;

    // The beam break
    private final BeamBreak m_beamBreak;

    // The Superstructure
    private final Superstructure m_superstructure;

    // Teleop drive command
    private final TeleopDriveCommand driveCommand;

    // Controllers
    private final SamuraiXboxController m_driverController =
        new SamuraiXboxController(OIConstants.kDriverControllerPort)
        .withDeadband(OIConstants.kControllerDeadband)
        .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);

    private final SamuraiXboxController m_secondaryController = 
        new SamuraiXboxController(OIConstants.kSecondaryControllerPort)
        .withDeadband(OIConstants.kControllerDeadband)
        .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_beamBreak = new BeamBreak(OIConstants.kBeamBreakPin);

        if (SystemConstants.currentMode == RobotMode.REAL) {
            m_arm = new ArmSubsystem(new ArmIOHardware());
            m_index = new IndexSubsystem(new IndexIOHardware());
            m_intake = new IntakeSubsystem(new IntakeIOHardware());
            m_shooter = new ShooterSubsystem(new ShooterIOHardware());

            m_drive = new DriveSubsystem(
                new GyroIOPigeon(),
                new ModuleIOHardware(ModuleConfig.FrontLeft), 
                new ModuleIOHardware(ModuleConfig.FrontRight), 
                new ModuleIOHardware(ModuleConfig.RearLeft), 
                new ModuleIOHardware(ModuleConfig.RearRight)
            );
        } else {
            m_arm = new ArmSubsystem(new ArmIODisabled());
            m_index = new IndexSubsystem(new IndexIODisabled());
            m_intake = new IntakeSubsystem(new IntakeIODisabled());
            m_shooter = new ShooterSubsystem(new ShooterIODisabled());

            // TODO: replace Drive with disabled?
            m_drive = new DriveSubsystem(
                new GyroIOPigeon(),
                new ModuleIOHardware(ModuleConfig.FrontLeft), 
                new ModuleIOHardware(ModuleConfig.FrontRight), 
                new ModuleIOHardware(ModuleConfig.RearLeft), 
                new ModuleIOHardware(ModuleConfig.RearRight)
            );
        }

        m_superstructure = new Superstructure(
            m_arm,
            m_index,
            m_intake,
            m_shooter,
            m_beamBreak.beamBrokenSupplier()
        );

        // TODO: check if driving is reversed
        driveCommand = m_drive.CommandBuilder.driveTeleop(
            () -> m_driverController.getLeftY(), 
            () -> m_driverController.getLeftX(), 
            () -> m_driverController.getRightX(),
            1,
            1,
            false);

        // Configure bindings
        configureDriverBindings();

        // Configure secondary controller bindings

    }

    /** Bind Triggers from the DriverController to Superstructure Commands. */
    private void configureDriverBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandbuilder();
        
        // TODO: check if all of these should be onTrue or whileTrue

        // Default command is teleop drive
        m_drive.setDefaultCommand(driveCommand);

        // Apply double clutch
        m_driverController.rightBumper()
            .whileTrue(driveCommand.applyDoubleClutch());

        // Apply single clutch
        m_driverController.leftBumper()
            .whileTrue(driveCommand.applySingleClutch());

        // Intake note
        m_driverController.leftTrigger()
            .whileTrue(superstructureCommands.intake());
        
        // Shoot note
        m_driverController.rightTrigger()
            .whileTrue(superstructureCommands.shoot());

        // Go to subwoofer position
        m_driverController.a()
            .onTrue(superstructureCommands.subwoofer());

        // Go to mid-low position
        m_driverController.b()
            .onTrue(superstructureCommands.midLow());

        // Go to mid-high position
        m_driverController.x()
            .onTrue(superstructureCommands.midHigh());

        // Go to amp posotion
        m_driverController.y()
            .onTrue(superstructureCommands.amp());
        
        // Manually move the arm up
        m_driverController.povUp()
            .whileTrue(superstructureCommands.armUpManual())
            .onFalse(superstructureCommands.detectMechanismState());

        // Manually move the arm down
        m_driverController.povDown()
            .whileTrue(superstructureCommands.armDownManual())
            .onFalse(superstructureCommands.detectMechanismState());

        // Force all rollers (intake, index, shooter) to run backward
        m_driverController.povLeft()
            .whileTrue(superstructureCommands.forceBackward())
            .onFalse(superstructureCommands.detectMechanismState());
        
        // Force all rollers (intake, index, shooter) to run forward
        m_driverController.povRight()
            .whileTrue(superstructureCommands.forceForward())
            .onFalse(superstructureCommands.detectMechanismState());

        // Re-zero the gyro
        m_driverController.start()
            .onTrue(
                Commands.runOnce(() -> m_drive.rezeroGyro())
            );
    }

    private void configureSecondaryControllerBindings(SecondaryControllerStates state) {
        DriveCommandFactory driveCommandBuilder = m_drive.getCommandBuilder();

        if (state == SecondaryControllerStates.DRIVETRAIN_SYSID_TRANS) {
            m_secondaryController.a()
                .and(m_secondaryController.x())
                .whileTrue(driveCommandBuilder.sysIdDyanmicTranslation(Direction.kForward));

            
        }
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
