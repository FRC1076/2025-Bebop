package frc.robot.subsystems.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.Control;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private ProfiledPIDController pidController;
    private boolean runPID = false;
    private double PIDTargetRadians = ArmConstants.kAbsoluteEncoderZero;

    private ArmFeedforward feedForwardController;

    public ArmSubsystem(ArmIO io) {
        this.io = io;
        pidController = new ProfiledPIDController(
            Control.kP,
            Control.kI,
            Control.kD,
            new Constraints(Control.kMaxVelocity, Control.kMaxAcceleration));
        feedForwardController = new ArmFeedforward(
            Control.kS,
            Control.kG, 
            Control.kV);
    }
    
    public void setVoltage(double volts) {
        if (inputs.positionRadians > ArmConstants.kMaxPositionRadians && volts > 0) {
            volts = 0;
        } else if (inputs.positionRadians < ArmConstants.kMinPositionRadians && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts /*+ feedForwardController.calculate(inputs.positionRadians, VELOCITY HERE) */);
    }

    public void setRunPID(boolean runPID) {
        this.runPID = runPID;
    }

    public void setPIDTarget(double targetRadians) {
        PIDTargetRadians = targetRadians;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (runPID) {
            setVoltage(pidController.calculate(inputs.positionRadians, PIDTargetRadians));
        }

        Logger.processInputs("Arm", inputs);
    }

    public Command startPID(double goalRadians) {
        return Commands.sequence(
            Commands.runOnce(() -> pidController.reset(inputs.positionRadians)),
            Commands.runOnce(() -> setPIDTarget(goalRadians)),
            Commands.runOnce(() -> setRunPID(true))
        );
    }

    public Command stopPID() {
        return runOnce(() -> setRunPID(false));
    }
}
