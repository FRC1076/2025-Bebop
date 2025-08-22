// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.arm;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ArmIOHardware implements ArmIO {
    private SparkMax m_leadMotor;
    private SparkMax m_followMotor;

    private SparkMaxConfig m_leadMotorConfig;
    private SparkMaxConfig m_followMotorConfig;

    private DutyCycleEncoder m_encoder;
    private RelativeEncoder m_relativeEncoder;

    public ArmIOHardware () {
        m_leadMotor = new SparkMax(ArmConstants.kLeadMotorCANId, MotorType.kBrushless);
        m_followMotor = new SparkMax(ArmConstants.kFollowMotorCANId, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(ArmConstants.kAbsoluteEncoderChannel);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        m_leadMotorConfig  
            .inverted(ArmConstants.kLeadMotorInverted)
            .smartCurrentLimit(ArmConstants.kCurrentLimitAmps)
            .idleMode(IdleMode.kBrake);

        m_followMotorConfig
            .follow(m_leadMotor)
            .inverted(ArmConstants.kLeadMotorInverted != ArmConstants.kFollowMotorInverted)
            .smartCurrentLimit(ArmConstants.kCurrentLimitAmps)
            .idleMode(IdleMode.kBrake);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_relativeEncoder = m_leadMotor.getEncoder();
    }

    /** Returns the position in radians, with adjustments to make sure that
     *  zero is in the right place and that the value can't loop around the full circle
     */
    private double getPositionRadians() {   
        return 
            (((m_encoder.get() * Math.PI * 2) // Raw value in radians
             + ArmConstants.kAbsoluteEncoderShift) % (2 * Math.PI))  // Add the shift to make sure we don't loop around to zero
             - ArmConstants.kAbsoluteEncoderZero; // Subtract the encoder zero to correct to the right value
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.appliedVoltage = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();

        inputs.leadMotorCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.followMotorCurrentAmps = m_followMotor.getOutputCurrent();

        inputs.positionRadians = getPositionRadians();
        inputs.velocityRadiansPerSecond = m_relativeEncoder.getVelocity();
    }
}
