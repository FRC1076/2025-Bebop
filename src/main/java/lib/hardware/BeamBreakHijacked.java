// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

/** Wrapper class for BeamBreaks */
public class BeamBreakHijacked extends BeamBreak {
    DigitalInput sensor;
    boolean state;

    public BeamBreakHijacked(int channel) {
        super(channel);
        state = false;
    }

    public DigitalInput getSensor() {
        return sensor;
    }

    public boolean isBeamBroken() {
        return state;
    }

    public BooleanSupplier beamBrokenSupplier() {
        return () -> state;
    }

    public void setBeamBroken(boolean state) {
        this.state = state;
    }
}
