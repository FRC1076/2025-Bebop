package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDState;

public class LEDIODigitalPins implements LEDIO {
    private final DigitalOutput m_pin1;
    private final DigitalOutput m_pin2;
    private final DigitalOutput m_pin3;

    public LEDIODigitalPins() {
        m_pin1 = new DigitalOutput(LEDConstants.kLEDPin1);
        m_pin2 = new DigitalOutput(LEDConstants.kLEDPin2);
        m_pin3 = new DigitalOutput(LEDConstants.kLEDPin3);
    }

    @Override
    public void setState(LEDState state) {
        m_pin1.set(state.pin1State);
        m_pin2.set(state.pin2State);
        m_pin3.set(state.pin3State);
    }
}
