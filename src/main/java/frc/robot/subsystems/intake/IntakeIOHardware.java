package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private final SparkMax m_Motor;
    private final SparkMaxConfig m_MotorConfig;

    public IntakeIOHardware() {
        m_Motor = new SparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);
        m_MotorConfig = new SparkMaxConfig();
        m_MotorConfig.inverted(false).smartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        m_Motor.configure(
            m_MotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        m_Motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = m_Motor.getAppliedOutput() * m_Motor.getBusVoltage();
        inputs.currentAmps = m_Motor.getOutputCurrent();
    }
}
