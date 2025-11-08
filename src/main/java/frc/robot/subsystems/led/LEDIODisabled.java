package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants.LEDState;

public class LEDIODisabled implements LEDIO {
    @Override
    public void setState(LEDState state) {
        return;
    }
}
