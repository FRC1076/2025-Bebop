package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double appliedVoltage = 0;

        public double leadMotorCurrentAmps = 0;
        public double followMotorCurrentAmps = 0;

        public double positionRadians = 0;
    }


    public abstract void setVoltage(double volts);

    public abstract void updateInputs(ArmIOInputs inputs);

}
