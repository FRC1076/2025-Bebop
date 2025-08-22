// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    /** Set the voltage of the motor */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /** Stop the motor from running */
    public void stop() {
        io.setVoltage(0);
    }

    /** Runs the motor at the desired voltage */
    public Command runVolts(double volts) {
        return Commands.run(
            () -> setVoltage(volts),
            this
        );
    }

    @Override 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
