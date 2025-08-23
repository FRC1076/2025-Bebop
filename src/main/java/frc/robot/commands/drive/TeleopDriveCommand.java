package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.doubleClutchRotationFactor;
import static frc.robot.Constants.DriveConstants.doubleClutchTranslationFactor;
import static frc.robot.Constants.DriveConstants.singleClutchRotationFactor;
import static frc.robot.Constants.DriveConstants.singleClutchTranslationFactor;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class TeleopDriveCommand extends Command {
    // Raw speed suppliers (ignore clutch)
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;
    private final DoubleSupplier rawOmegaSupplier;

    // Speed suppliers after clutch applied
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    // The subsystem
    private final DriveSubsystem m_drive;

    // A consumer to accept the ChassisSpeeds
    private Consumer<ChassisSpeeds> driveConsumer;

    // The clutch factors
    private double transClutchFactor = 1.0;
    private double rotClutchFactor = 1.0;

    public TeleopDriveCommand(
        DriveSubsystem drive,
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier,
        boolean useSpeedScaling
    ) {
        if(useSpeedScaling) {
            this.rawXSupplier = () -> scaleSpeed(xSupplier.getAsDouble(), xSupplier, ySupplier) * DriveConstants.maxTranslationSpeedMPS;
            this.rawYSupplier = () -> scaleSpeed(ySupplier.getAsDouble(), xSupplier, ySupplier) * DriveConstants.maxTranslationSpeedMPS;
        } else {
            this.rawXSupplier = () -> xSupplier.getAsDouble() * DriveConstants.maxTranslationSpeedMPS;
            this.rawYSupplier = () -> ySupplier.getAsDouble() * DriveConstants.maxTranslationSpeedMPS;
        }

        this.rawOmegaSupplier = () -> omegaSupplier.getAsDouble() * DriveConstants.maxRotationSpeedRadPerSec;

        this.m_drive = drive;

        reloadCommand();
        addRequirements(drive);
    }

    /** Updates command with new values */
    private void reloadCommand() {
        xSupplier = () -> rawXSupplier.getAsDouble() * transClutchFactor;
        ySupplier = () -> rawYSupplier.getAsDouble() * transClutchFactor;
        omegaSupplier = () -> rawOmegaSupplier.getAsDouble() * rotClutchFactor;

        driveConsumer = m_drive::driveCLFO;
    }

    /** Scale the speed to be between -1.0 and 1.0 */
    public double scaleSpeed(double speed, DoubleSupplier xSupplier, DoubleSupplier ySupplier){
        return speed / Math.max(Math.sqrt(Math.pow(xSupplier.getAsDouble(), 2) + Math.pow(ySupplier.getAsDouble(), 2)), 1);
    }

    /** Set the clutch factors and reload the Command */
    public void setClutchFactors(double transFactor, double rotFactor) {
        this.transClutchFactor = transFactor;
        this.rotClutchFactor = rotFactor;
        reloadCommand();
    }

    @Override
    public void execute(){
        ChassisSpeeds speeds = new ChassisSpeeds(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble(),
            omegaSupplier.getAsDouble()
        );
        driveConsumer.accept(speeds);
    }

    // Automatically cancels drive command when teleop is not enabled
    @Override
    public boolean isFinished(){
        return !DriverStation.isTeleopEnabled();
    }

    //** Returns a command that applies a clutch to the TeleopDriveCommand */
    public Command applyClutchFactors(double transFactor, double rotFactor){
        return Commands.startEnd(
            () -> this.setClutchFactors(transFactor, rotFactor), 
            () -> this.setClutchFactors(1.0, 1.0)
        );
    }

    //** Returns a command that applies a single clutch to the TeleopDriveCommand */
    public Command applySingleClutch() {
        return applyClutchFactors(singleClutchTranslationFactor, singleClutchRotationFactor);
    }

    //** Returns a command that applies a double clutch to the TeleopDriveCommand */
    public Command applyDoubleClutch() {
        return applyClutchFactors(doubleClutchTranslationFactor, doubleClutchRotationFactor);
    }
}
