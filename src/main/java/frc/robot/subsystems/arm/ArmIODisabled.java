// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

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