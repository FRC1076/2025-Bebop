package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
    private final IndexIO io;
    private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

    public IndexSubsystem(IndexIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.setVoltage(0);
    }

    public Command runVolts(double volts) {
        return Commands.run(
            () -> setVoltage(volts),
            this
        );
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Index", inputs);
    }
}
