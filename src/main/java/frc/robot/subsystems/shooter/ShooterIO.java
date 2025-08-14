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
    
    public abstract void setVoltage(double leftMotorVolts, double rightMotorVolts);

    public abstract void updateInputs(ShooterIOInputs inputs);
}