// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// The contents of this file are based off those of
// team 6328 Mechanical Advantage's Spark Swerve Template,
// whose license can be found in AdvantageKit-License.md
// in the root directory of this file

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.moduleTranslations;
import static frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive.MaxModuleSpeed;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.TeleopDriveCommandV2;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class DriveSubsystem extends SubsystemBase {
    private final GyroIO gyroIO;
    private final Module[] modules = new Module[4];
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    }; // For delta tracking

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    public final DriveCommandFactory CommandBuilder;

    public DriveSubsystem(
        GyroIO gyroIO,
        ModuleIO FLModuleIO,
        ModuleIO FRModuleIO,
        ModuleIO RLModuleIO,
        ModuleIO RRModuleIO
    ){
        this.gyroIO = gyroIO;
        modules[0] = new Module(FLModuleIO,"FrontLeft");
        modules[1] = new Module(FRModuleIO,"FrontRight");
        modules[2] = new Module(RLModuleIO, "RearLeft");
        modules[3] = new Module(RRModuleIO, "RearRight");
        

        OdometryThread.getInstance().start();

        CommandBuilder = new DriveCommandFactory(this);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            states[i] = modules[i].getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose){
        for(Module module : modules) {
            module.resetTurnRelativeEncoder();
        }
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), newPose);
    }

    /** Chassis-oriented Closed-Loop driving */
    public void driveCLCO(ChassisSpeeds speeds){
        ChassisSpeeds discSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i]);
        }

        SwerveModuleState[] actualStates = {modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()};
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/Actual", actualStates);
    }

    /** Field-oriented Closed-loop driving */
    public void driveCLFO(ChassisSpeeds speeds){
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rawGyroRotation); // TODO: does this need to be flipped based on alliance?
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i]);
        }

        SwerveModuleState[] actualStates = {modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()};
        Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
        Logger.recordOutput("SwerveStates/Actual", actualStates);
    }

    /** Reset the current yaw heading of the gyro to zero */
    public void rezeroGyro() {
        gyroIO.reset();
    }

    @Override
    public void periodic(){
        
        //MUST BE CALLED BEFORE CONSUMING DATA FROM ODOMETRY THREAD
        OdometryThread.getInstance().poll();

        if (DriverStation.isDisabled()) {
            for (Module module : modules) {
                module.stop();
            }
        }
        
        // Update gyro logging
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        //Update module logging, process odometry
        for (Module module : modules) {
            module.periodic();
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++){
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int modIndex = 0; modIndex < 4; modIndex++){
                modulePositions[modIndex] = modules[modIndex].getOdometryModulePositions()[i];
                moduleDeltas[modIndex] = new SwerveModulePosition(
                    modulePositions[modIndex].distanceMeters - lastModulePositions[modIndex].distanceMeters,
                    modulePositions[modIndex].angle);
                lastModulePositions[modIndex] = modulePositions[modIndex];
            }
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }
            poseEstimator.updateWithTime(sampleTimestamps[i],rawGyroRotation,modulePositions);
        }
        rawGyroRotation = gyroInputs.yawPosition;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /* EVERYTHING SYSID */
    private void runTranslationCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runTranslationCharacterization(output);
        }
    }

    private void runSpinCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runSpinCharacterization(output);
        }
    }

    /** Translation SysID */
    private final SysIdRoutine m_SysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> Logger.recordOutput("Drive/sysIdStateTranslation", state.toString()) // Log state to AKit
        ),
        new SysIdRoutine.Mechanism(
            output -> runTranslationCharacterization(output.in(Volts)), null, this)
    );

    /** Rotation SysID (spins motors only) */
    private final SysIdRoutine m_SysIdRoutineSpin = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> Logger.recordOutput("Drive/sysIdStateSpin", state.toString()) // Log state to AKit
        ),
        new SysIdRoutine.Mechanism(
            output -> runSpinCharacterization(output.in(Volts)), null, this)
    );

    public DriveCommandFactory getCommandBuilder() {
        return CommandBuilder;
    }

    /** Builds the drive commands */
    public class DriveCommandFactory {
        public DriveSubsystem drive;

        private DriveCommandFactory(DriveSubsystem drive) {
            this.drive = drive;
        }

        public TeleopDriveCommandV2 driveTeleop(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double transClutchFactor, double rotClutchFactor, boolean useSpeedScaling) {
            TeleopDriveCommandV2 driveCommand = new TeleopDriveCommandV2(drive, drive::driveCLFO, xSupplier, ySupplier, omegaSupplier, useSpeedScaling);
            driveCommand.setClutchFactors(transClutchFactor, rotClutchFactor);
            return driveCommand;
        }

        public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
            return m_SysIdRoutineTranslation.quasistatic(direction);
        }

        public Command sysIdDyanmicTranslation(SysIdRoutine.Direction direction) {
            return m_SysIdRoutineTranslation.dynamic(direction);
        }

        public Command sysIdQuasistaticSpin(SysIdRoutine.Direction direction) {
            return m_SysIdRoutineSpin.quasistatic(direction);
        }

        public Command sysIdDyanmicSpin(SysIdRoutine.Direction direction) {
            return m_SysIdRoutineSpin.dynamic(direction);
        }
    }
}