// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

/** Wrapper class for BeamBreaks */
public class BeamBreak {
    DigitalInput sensor;

    public BeamBreak(int channel) {
        sensor = new DigitalInput(channel);
    }

    public DigitalInput getSensor() {
        return sensor;
    }

    public boolean isBeamBroken() {
        return !sensor.get();
    }

    public BooleanSupplier beamBrokenSupplier() {
        return () -> !sensor.get();
    }
}
