package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.doubleClutchRotationFactor;
import static frc.robot.Constants.DriveConstants.doubleClutchTranslationFactor;
import static frc.robot.Constants.DriveConstants.maxRotationSpeedRadPerSec;
import static frc.robot.Constants.DriveConstants.maxTranslationSpeedMPS;
import static frc.robot.Constants.DriveConstants.singleClutchRotationFactor;
import static frc.robot.Constants.DriveConstants.singleClutchTranslationFactor;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TeleopDriveCommandV2 extends Command {
    // Raw speed suppliers from controller
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;
    private final DoubleSupplier rawOmegaSupplier;

    // The subsystem
    private final DriveSubsystem m_drive;

    // Drive consumer
    private Consumer<ChassisSpeeds> driveConsumer;

    // Clutch factors
    private double transClutchFactor = 1;
    private double rotClutchFactor = 1;

    // Whether to use speed scaling
    private final boolean useSpeedScaling;

    public TeleopDriveCommandV2(
        DriveSubsystem drive,
        Consumer<ChassisSpeeds> driveConsumer,
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier,
        boolean useSpeedScaling
    ) {
        this.m_drive = drive;
        this.driveConsumer = driveConsumer;
        this.rawXSupplier = xSupplier;
        this.rawYSupplier = ySupplier;
        this.rawOmegaSupplier = omegaSupplier;
        this.useSpeedScaling = useSpeedScaling;

        addRequirements(m_drive);
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = Math.hypot(x, y);
        Rotation2d linearDirection = Rotation2d.fromRadians(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Apply clutches
        linearMagnitude *= transClutchFactor;

        if (useSpeedScaling) {
            // Caps linear magnitude before multiplication by maximum speed at 1 instead of sqrt(2)
            linearMagnitude = Math.min(linearMagnitude, 1);
        }

        linearMagnitude *= maxTranslationSpeedMPS;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
            .getTranslation();
    }

    @Override
    public void execute() {
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(rawXSupplier.getAsDouble(), rawYSupplier.getAsDouble());

        double omegaRaw = rawOmegaSupplier.getAsDouble();
        double omega = Math.copySign(omegaRaw * omegaRaw, omegaRaw) * rotClutchFactor;

        driveConsumer.accept(
            new ChassisSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(),
                omega * maxRotationSpeedRadPerSec
            )
        );
    }

    // Automatically cancels drive command when teleop is not enabled
    @Override
    public boolean isFinished(){
        return !DriverStation.isTeleopEnabled();
    }

    /** Set the clutch factors and reload the Command */
    public void setClutchFactors(double transFactor, double rotFactor) {
        this.transClutchFactor = transFactor;
        this.rotClutchFactor = rotFactor;
    }

    /** Returns a command that applies a clutch to the TeleopDriveCommand */
    public Command applyClutchFactors(double transFactor, double rotFactor){
        return Commands.startEnd(
            () -> this.setClutchFactors(transFactor, rotFactor), 
            () -> this.setClutchFactors(1.0, 1.0)
        );
    }

    /** Returns a command that applies a single clutch to the TeleopDriveCommand */
    public Command applySingleClutch() {
        return applyClutchFactors(singleClutchTranslationFactor, singleClutchRotationFactor);
    }

    /** Returns a command that applies a double clutch to the TeleopDriveCommand */
    public Command applyDoubleClutch() {
        return applyClutchFactors(doubleClutchTranslationFactor, doubleClutchRotationFactor);
    }
}
