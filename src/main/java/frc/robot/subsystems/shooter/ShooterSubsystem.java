// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants.Control;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private PIDController leftPidController;
    private PIDController rightPidController;
    private double leftPidTargetRadPerSec = 0;
    private double rightPidTargetRadPerSec = 0;
    private boolean runPid = false;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        leftPidController = new PIDController(
            Control.kPLeft,
            Control.kILeft,
            Control.kDLeft
        );

        rightPidController = new PIDController(
            Control.kPRight,
            Control.kIRight,
            Control.kDRight
        );
    }

    public void setVoltage(double leftMotorVoltage, double rightMotorvoltage) {
        io.setVoltage(leftMotorVoltage, rightMotorvoltage);
    }

    public void setRunPID(boolean run) {
        runPid = run;
    }

    public void setPIDTarget(double leftMotorTargetRadPerSec, double rightMotorTargetRadPerSec) {
        leftPidTargetRadPerSec = leftMotorTargetRadPerSec;
        rightPidTargetRadPerSec = rightMotorTargetRadPerSec;
    }
    
    public void stop() {
        io.setVoltage(0, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        inputs.pidRunning = runPid;
        if (runPid) {
            inputs.leftMotorPidTargetRadPerSec = leftPidTargetRadPerSec;
            inputs.rightMotorPidTargetRadPerSec = rightPidTargetRadPerSec;

            leftPidController.calculate(inputs.leftEncoderVelocityRadPerSec, leftPidTargetRadPerSec);
            rightPidController.calculate(inputs.rightEncoderVelocityRadPerSec, rightPidTargetRadPerSec);
        }

        Logger.processInputs("Shooter", inputs);
    }

    public Command startPid(double leftMotorTargetRadPerSec, double rightMotorTargetRadPerSec) {
        return Commands.sequence(
            Commands.runOnce(() -> setPIDTarget(leftMotorTargetRadPerSec, rightMotorTargetRadPerSec)), 
            Commands.runOnce(() -> setRunPID(true))
        );
    }
    
    public Command stopPid() {
        return Commands.runOnce(() -> setRunPID(false));
    }
}