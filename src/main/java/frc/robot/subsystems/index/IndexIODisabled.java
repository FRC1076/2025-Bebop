package frc.robot.subsystems.index;

public class IndexIODisabled implements IndexIO {
    double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void updateInputs(IndexIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
    }
}
