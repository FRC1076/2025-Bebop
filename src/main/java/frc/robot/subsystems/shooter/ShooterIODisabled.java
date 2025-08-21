package frc.robot.subsystems.shooter;

public class ShooterIODisabled implements ShooterIO {
    double leftMotorVoltageTarget = 0;
    double rightMotorVoltageTarget = 0;

    @Override
    public void setVoltage(double leftMotorVolts, double rightMotorVolts) {
        leftMotorVoltageTarget = leftMotorVolts;
        rightMotorVoltageTarget = rightMotorVolts;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = leftMotorVoltageTarget;
        inputs.rightMotorAppliedVoltage = rightMotorVoltageTarget;
    }
}
