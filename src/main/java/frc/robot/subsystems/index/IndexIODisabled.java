// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

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
