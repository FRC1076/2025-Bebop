// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants.Control;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private PIDController leftPidController;
    private PIDController rightPidController;
    private SimpleMotorFeedforward ffController;
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

        ffController = new SimpleMotorFeedforward(
            Control.kS,
            Control.kV,
            Control.kA
        );
    }

    /** Set the voltage of the left motor and the right motor */
    public void setVoltage(double leftMotorVoltage, double rightMotorvoltage) {
        io.setVoltage(leftMotorVoltage, rightMotorvoltage);
    }

    /** Set whether or not the PID is on
     * 
     * @param run Whether or not to run the PID
    */
    public void setRunPID(boolean run) {
        runPid = run;
    }

    /** Sets the target of the PID controller (but doesn't turn it on) */
    public void setPIDTarget(double leftMotorTargetRadPerSec, double rightMotorTargetRadPerSec) {
        leftPidTargetRadPerSec = leftMotorTargetRadPerSec;
        rightPidTargetRadPerSec = rightMotorTargetRadPerSec;
    }
    
    /** Stop both motors */
    public void stop() {
        io.setVoltage(0, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        inputs.pidRunning = runPid;
        if (runPid && leftPidTargetRadPerSec != 0 && rightPidTargetRadPerSec != 0) {
            inputs.leftMotorPidTargetRadPerSec = leftPidTargetRadPerSec;
            inputs.rightMotorPidTargetRadPerSec = rightPidTargetRadPerSec;

            setVoltage(
                rightPidController.calculate(inputs.rightEncoderVelocityRadPerSec, rightPidTargetRadPerSec) + ffController.calculate(rightPidTargetRadPerSec),
                leftPidController.calculate(inputs.leftEncoderVelocityRadPerSec, leftPidTargetRadPerSec) + ffController.calculate(leftPidTargetRadPerSec)
            );
            
        } else if (runPid) {
            stop();
        }

        Logger.processInputs("Shooter", inputs);
    }

    /** Starts the PID controllers with the desired targets
     * 
     * @param leftMotorTargetRadPerSec Target speed of the left motor in radians per second
     * @param rightMotorTargetRadPerSec Target speed of the right motor in radians per second
     */
    public Command startPid(double leftMotorTargetRadPerSec, double rightMotorTargetRadPerSec) {
        return Commands.sequence(
            Commands.runOnce(() -> setPIDTarget(leftMotorTargetRadPerSec, rightMotorTargetRadPerSec)), 
            Commands.runOnce(() -> setRunPID(true))
        );
    }
    
    /** Stops the PID controllers */
    public Command stopPid() {
        return Commands.runOnce(() -> setRunPID(false));
    }
}