package frc.robot.subsystems.index;

import frc.robot.Constants.IndexConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexIOHardware implements IndexIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    public IndexIOHardware() {
        m_motor = new SparkMax(IndexConstants.kIndexMotorCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();
        m_motorConfig
            .inverted(IndexConstants.kIndexMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IndexConstants.kIndexMotorCurrentLimit);

        m_motor.configure(
            m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IndexIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
    }
}