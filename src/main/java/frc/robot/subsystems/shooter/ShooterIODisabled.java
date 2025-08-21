// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

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
