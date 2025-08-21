package frc.robot.subsystems.intake;

public class IntakeIODisabled implements IntakeIO {
    double voltageTarget = 0;
    
    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
    }
}
