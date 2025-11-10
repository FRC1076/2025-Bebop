package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants.LEDState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final LEDIO io;
    private LEDState currentState = LEDState.PURPLE;
    private LEDState previousState = LEDState.PURPLE;

    public LEDSubsystem(LEDIO io) {
        this.io = io;
    }

    public void setState(LEDState state) {
        previousState = currentState;
        currentState = state;

        io.setState(state);
    }

    public Command setStateCommand(LEDState state) {
        return Commands.runOnce(
            () -> setState(state),
            this
        ).ignoringDisable(true);
    }

    public Command setTempStateTimed(LEDState state, double seconds) {
        return Commands.startEnd(
            () -> setState(state), 
            () -> setState(previousState),
            this
        ).withTimeout(seconds).ignoringDisable(true);
    }
}
