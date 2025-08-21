// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    public IntakeIOHardware() {
        m_motor = new SparkMax(IntakeConstants.kIntakeMotorCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();
        m_motorConfig
            .inverted(IntakeConstants.kIntakeMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        m_motor.configure(
            m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
    }
}
