// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ShooterIOHardware implements ShooterIO {
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;

    private SparkMaxConfig m_leftMotorConfig;
    private SparkMaxConfig m_rightMotorConfig;

    private RelativeEncoder m_leftMotorEncoder;
    private RelativeEncoder m_rightMotorEncoder;

    public ShooterIOHardware() {
        m_leftMotor = new SparkMax(ShooterConstants.kLeftMotorCanId, MotorType.kBrushless);
        m_rightMotor = new SparkMax(ShooterConstants.kRightMotorCanId, MotorType.kBrushless);

        m_leftMotorConfig = new SparkMaxConfig();
        m_leftMotorConfig
            .inverted(ShooterConstants.kLeftMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
            .voltageCompensation(12)
            .encoder
                .velocityConversionFactor(ShooterConstants.kVelocityConversionFactor);

        m_leftMotor.configure(
            m_leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_rightMotorConfig = new SparkMaxConfig();
        m_rightMotorConfig
            .inverted(ShooterConstants.kRightMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
            .voltageCompensation(12)
            .encoder
                .velocityConversionFactor(ShooterConstants.kVelocityConversionFactor);

        m_rightMotor.configure(
            m_rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_leftMotorEncoder = m_leftMotor.getEncoder();
        m_rightMotorEncoder = m_rightMotor.getEncoder();
    }

    @Override
    public void setVoltage(double leftMotorVolts, double rightMotorVolts) {
        m_leftMotor.setVoltage(leftMotorVolts);
        m_rightMotor.setVoltage(rightMotorVolts);
    }
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage();
        inputs.rightMotorAppliedVoltage = m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage();
        
        inputs.leftMotorCurrentAmps = m_leftMotor.getOutputCurrent();
        inputs.rightMotorCurrentAmps = m_rightMotor.getOutputCurrent();

        inputs.leftEncoderVelocityRadPerSec = m_leftMotorEncoder.getVelocity();
        inputs.rightEncoderVelocityRadPerSec = m_rightMotorEncoder.getVelocity();
    }
}