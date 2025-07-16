package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {
    @AutoLog
    public static class IndexIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void updateInputs(IndexIOInputs inputs);

}
