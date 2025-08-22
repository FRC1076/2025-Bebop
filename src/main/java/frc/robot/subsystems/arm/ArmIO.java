// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double appliedVoltage = 0;

        public double leadMotorCurrentAmps = 0;
        public double followMotorCurrentAmps = 0;

        public double positionRadians = 0;
        public double velocityRadiansPerSecond = 0;

        public double pidTargetRadians = 0;
        public boolean pidRunning = false;
    }

    /** Set the voltage of the motor */
    public abstract void setVoltage(double volts);

    public abstract void updateInputs(ArmIOInputs inputs);

}
