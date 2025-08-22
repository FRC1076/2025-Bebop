// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

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

    /** Sets the voltage of the motor */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /** Stops the motor from running (sets the voltage to zero) */
    public void stop() {
        io.setVoltage(0);
    }

    /** Runs a voltage on the motor */
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
