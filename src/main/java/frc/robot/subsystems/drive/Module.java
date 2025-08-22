// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// The contents of this file are based off those of
// team 6328 Mechanical Advantage's Spark Swerve Template,
// whose license can be found in AdvantageKit-License.md
// in the root directory of this file

package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private final ModuleIO io;
    private final String ID;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final SimpleMotorFeedforward driveFFController;
    private SwerveModulePosition[] odometryModulePositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, String ID){
        this.io = io;
        this.ID = ID;
        driveFFController = new SimpleMotorFeedforward(
            Drive.kS,
            Drive.kV,
            Drive.kA,
            0.2
        );
    }

    /*Sets desired state in closed-loop mode */
    public void setDesiredState(SwerveModuleState state){
        //io.updateInputs(inputs); //Fetches latest data from IO layer
        state.optimize(inputs.turnPosition);
        io.setDriveVelocity(
            state.speedMetersPerSecond, 
            driveFFController.calculate(state.speedMetersPerSecond)
        );
        io.setTurnPosition(
            state.angle.getRotations(), 
            0
        );
    }

    /* Sets desired state with overridden Feedforward */
    public void setDesiredState(SwerveModuleState state, double driveFFVolts, double turnFFVolts){
        //io.updateInputs(inputs); //Fetches latest data from IO layer
        state.optimize(inputs.turnPosition);
        io.setDriveVelocity(
            state.speedMetersPerSecond, 
            driveFFVolts
        );
        io.setTurnPosition(
            state.angle.getRotations(), 
            turnFFVolts
        );
    }

    public void stop(){
        io.setDriveVolts(0.0);
        io.setTurnVolts(0.0);
    }

    public double[] getOdometryTimestamps(){
        return inputs.odometryTimestamps;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
    }

    public SwerveModulePosition[] getOdometryModulePositions(){
        return odometryModulePositions;
    }

    /** Must be manually called by DriveSubsystem. WARNING: NOT THREAD-SAFE, SHOULD ONLY BE CALLED FROM MAIN THREAD */
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + ID, inputs);
        int sampleCount = OdometryThread.getInstance().sampleCount;
        odometryModulePositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++){
            odometryModulePositions[i] = new SwerveModulePosition(inputs.odometryDrivePositionsMeters[i],inputs.odometryTurnPositions[i]);
        }
    }
}