package frc.robot.subsystems.arm;

public class ArmIODisabled implements ArmIO {
    double voltageTarget = 0;
    
    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
    }
}