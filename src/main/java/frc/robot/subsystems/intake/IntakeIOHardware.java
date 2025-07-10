package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    public IntakeIOHardware() {
        m_motor = new SparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();
        m_motorConfig.inverted(false).smartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

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
