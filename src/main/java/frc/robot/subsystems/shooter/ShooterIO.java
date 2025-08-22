// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leftMotorAppliedVoltage = 0;
        public double rightMotorAppliedVoltage = 0;

        public double leftMotorCurrentAmps = 0;
        public double rightMotorCurrentAmps = 0;

        public double leftEncoderVelocityRadPerSec = 0;
        public double rightEncoderVelocityRadPerSec = 0;

        public double leftMotorPidTargetRadPerSec = 0;
        public double rightMotorPidTargetRadPerSec = 0;

        public boolean pidRunning = false;
    }
    
    /** Set the desired voltages of the left and right motors */
    public abstract void setVoltage(double leftMotorVolts, double rightMotorVolts);

    public abstract void updateInputs(ShooterIOInputs inputs);
}