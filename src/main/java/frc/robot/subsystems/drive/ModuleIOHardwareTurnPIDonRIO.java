// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// The contents of this file are based off those of
// team 6328 Mechanical Advantage's Spark Swerve Template,
// whose license can be found in AdvantageKit-License.md
// in the root directory of this file

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import frc.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Turn;
import frc.robot.utils.SignalUtils;
import static frc.robot.Constants.DriveConstants.odometryFrequencyHz;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class ModuleIOHardwareTurnPIDonRIO implements ModuleIO {
    
    private final SparkMax m_turnMotor;
    private final PIDController TurnPID;
    private final RelativeEncoder TurnRelEncoder;

    // Stuff for the turn PID controller
    private double turnPIDTarget = 0;
    private double turnFFVolts = 0;
    private boolean turnPIDEnabled = false;

    private final SparkMax m_driveMotor;
    private final SparkClosedLoopController DrivePID;
    private final RelativeEncoder DriveRelEncoder;

    private final ConcurrentLinkedQueue<Long> timestampQueue;
    private final ConcurrentLinkedQueue<Double> drivePositionQueue;
    private final ConcurrentLinkedQueue<Double> turnPositionQueue;

    private final ArrayList<Long> timestampBuffer = new ArrayList<>();
    private final ArrayList<Double> drivePositionBuffer = new ArrayList<>();
    private final ArrayList<Double> turnPositionBuffer = new ArrayList<>();

    private final CANcoder m_turnEncoder;
    private final StatusSignal<Angle> turnAbsolutePosition;

    public ModuleIOHardwareTurnPIDonRIO(ModuleConfig config){

        m_turnMotor = new SparkMax(config.TurnPort, MotorType.kBrushless);
        TurnPID = new PIDController(
            Turn.kP,
            Turn.kI,
            Turn.kD
        );
        TurnRelEncoder = m_turnMotor.getEncoder();

        m_driveMotor = new SparkMax(config.DrivePort, MotorType.kBrushless);
        DrivePID = m_driveMotor.getClosedLoopController();
        DriveRelEncoder = m_driveMotor.getEncoder();
    
        m_turnEncoder = new CANcoder(config.EncoderPort);

        //Config turn absolute encoder here
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.FutureProofConfigs = false;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // Originally 1
        encoderConfig.MagnetSensor.MagnetOffset = config.EncoderOffsetRots;
        m_turnEncoder.getConfigurator().apply(encoderConfig);
        turnAbsolutePosition = m_turnEncoder.getAbsolutePosition();

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Turn.CurrentLimit)
            .voltageCompensation(Turn.VoltageCompensation);
        turnConfig
            .encoder
            .positionConversionFactor(Turn.PositionConversionFactor)
            .velocityConversionFactor(Turn.VelocityConversionFactor);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Turn.kP)
            .i(Turn.kI)
            .d(Turn.kD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI); // Originally: 0, 2*Math.PI 
        turnConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/odometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        TurnRelEncoder.setPosition(turnAbsolutePosition.getValueAsDouble() * Turn.PositionConversionFactor);
        
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Drive.CurrentLimit)
            .voltageCompensation(Drive.VoltageCompensation);
        driveConfig
            .encoder
            .positionConversionFactor(Drive.PositionConversionFactor)
            .velocityConversionFactor(Drive.VelocityConversionFactor);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Drive.kP)
            .i(Drive.kI)
            .d(Drive.kD);
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/odometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        DriveRelEncoder.setPosition(0.0);
        
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(SignalUtils.getSparkMaxPositionSignal(m_driveMotor));
        OdometryThread.getInstance().registerErrorSignal(SignalUtils.getSparkMaxErrorSignal(m_driveMotor));
        turnPositionQueue = OdometryThread.getInstance().registerSignal(SignalUtils.getSparkMaxPositionSignal(m_turnMotor));
        OdometryThread.getInstance().registerErrorSignal(SignalUtils.getSparkMaxErrorSignal(m_turnMotor));
    }

    @Override
    public void periodic() {
        turnAbsolutePosition.refresh();

        if (turnPIDEnabled) {
            setTurnVoltsKeepPIDOn(
                TurnPID.calculate(
                    MathUtil.angleModulus(turnAbsolutePosition.getValueAsDouble() * Turn.PositionConversionFactor),
                    turnPIDTarget
                ) + turnFFVolts
            );
        }
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){

        inputs.drivePositionMeters = DriveRelEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = DriveRelEncoder.getVelocity();
        inputs.driveAppliedVolts = m_driveMotor.getBusVoltage() * m_driveMotor.getAppliedOutput();
        inputs.driveCurrentAmps = m_driveMotor.getOutputCurrent();
        
        inputs.turnAbsolutePositionRadians = turnAbsolutePosition.getValueAsDouble() * Turn.PositionConversionFactor;
        inputs.turnPosition = turnAbsolutePosition.getValueAsDouble() * Turn.PositionConversionFactor;
        inputs.turnVelocityRadiansPerSecond = TurnRelEncoder.getVelocity();
        inputs.turnAppliedVolts = m_turnMotor.getBusVoltage() * m_turnMotor.getAppliedOutput();
        inputs.turnCurrentAmps = m_turnMotor.getOutputCurrent();

        /** Should be called after poll() in the main thread */
        int samples = OdometryThread.getInstance().sampleCount;
        OdometryThread.safeDrain(timestampQueue, timestampBuffer,samples);
        OdometryThread.safeDrain(drivePositionQueue, drivePositionBuffer, samples);
        OdometryThread.safeDrain(turnPositionQueue, turnPositionBuffer, samples);
        inputs.odometryTimestamps = timestampBuffer.stream().mapToDouble((Long value) -> value/1e6).toArray();
        inputs.odometryDrivePositionsMeters = drivePositionBuffer.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionBuffer.stream().map((Double value) -> Rotation2d.fromRadians(value)).toArray(Rotation2d[]::new);
        timestampBuffer.clear();
        drivePositionBuffer.clear();
        turnPositionBuffer.clear();
    }

    @Override
    public void setDriveVolts(double volts){
        m_driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVolts(double volts){
        turnPIDEnabled = false;
        m_turnMotor.setVoltage(volts);
    }

    public void setTurnVoltsKeepPIDOn(double volts) {
        m_turnMotor.setVoltage(volts);
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec, double FFVolts){
        DrivePID.setReference(
            velocityMetersPerSec,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            FFVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(double positionRadians, double FFVolts){
        turnPIDTarget = MathUtil.angleModulus(positionRadians);
        turnFFVolts = FFVolts;
        turnPIDEnabled = true;
    }
    
}