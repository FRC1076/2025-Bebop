// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.Control;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ArmSubsystem extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged(); // TODO: Add PID setpoint to logged inputs

    private ProfiledPIDController pidController;
    private boolean runPID = false;
    private double PIDTargetRadians = ArmConstants.kAbsoluteEncoderZero;

    private ArmFeedforward feedForwardController;

    private SysIdRoutine sysid;

    public ArmSubsystem(ArmIO io) {
        this.io = io;

        pidController = new ProfiledPIDController(
            Control.kP,
            Control.kI,
            Control.kD,
            new Constraints(Control.kMaxVelocity, Control.kMaxAcceleration)
        );

        feedForwardController = new ArmFeedforward(
            Control.kS,
            Control.kG, 
            Control.kV
        );

        sysid = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(2),
                null,
                (state) -> Logger.recordOutput("Arm/SysIDState", state.toString())
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );
    }
    
    /** Sets the voltage of the motors.
     *  Includes one way software stops
     *  (will not run up past the maximum position or down below the minimum position)
     */
    public void setVoltage(double volts) {
        if (inputs.positionRadians > ArmConstants.kMaxPositionRadians && volts > 0) {
            volts = 0;
        } else if (inputs.positionRadians < ArmConstants.kMinPositionRadians && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts + feedForwardController.calculate(inputs.positionRadians, inputs.velocityRadiansPerSecond));
    }

    /** Returns the position of the arms in radians */
    public double getPosition() {
        return inputs.positionRadians;
    }

    /** Sets whether or not to run the PID controller */
    public void setRunPid(boolean runPID) {
        this.runPID = runPID;
    }

    /** Sets the target of the PID (but doesn't run it) */
    public void setPidTarget(double targetRadians) {
        PIDTargetRadians = targetRadians;
    }

    /** Gets whether or not the arm is within the tolerance of the PID's target.
     * 
     * @param toleranceRadians The tolerance in radians
     */
    public boolean withinTolerance(double toleranceRadians) {
        return Math.abs(PIDTargetRadians - inputs.positionRadians) < toleranceRadians;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        inputs.pidRunning = runPID;

        if (runPID) {
            setVoltage(pidController.calculate(inputs.positionRadians, PIDTargetRadians));

            inputs.pidTargetRadians = PIDTargetRadians;
        }

        Logger.processInputs("Arm", inputs);
    }

    /** Starts the PID controller with the desired target
     * 
     * @param goalRadians The PID's target in radians
     */
    public Command startPid(double goalRadians) {
        return Commands.sequence(
            Commands.runOnce(() -> pidController.reset(inputs.positionRadians)),
            Commands.runOnce(() -> setPidTarget(goalRadians)),
            Commands.runOnce(() -> setRunPid(true))
        );
    }

    /** Stops the PID controller */
    public Command stopPID() {
        return runOnce(() -> setRunPid(false));
    }

    /** Runs volts directly */
    public Command runVolts(double volts) {
        return Commands.runOnce(() -> setVoltage(volts));
    }

    /** Runs a quasistatic SysID routine in the specified direction */
    public Command armSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    /** Runs a dynamic SysID routine in the specified direction */
    public Command armSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }
}
