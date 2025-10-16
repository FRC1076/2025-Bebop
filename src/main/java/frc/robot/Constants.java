// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;

        public static final int kBeamBreakPin = 5;
    }

    public static class SystemConstants {
        public static final RobotMode currentMode = RobotMode.REAL;

        public static final boolean enableSignalLogger = false;
        public static final boolean increaseThreadPriority = true;
        
        public static enum RobotMode {
            REAL,
            SIM,
            REPLAY;
        }
    }

    public static class IntakeConstants {
        public static final int kIntakeMotorCANId = 5;
        public static final boolean kIntakeMotorInverted = false;
        public static final int kIntakeMotorCurrentLimit = 20;
    }
    
    public static class ArmConstants {
        public static final int kLeadMotorCANId = 6;
        public static final int kFollowMotorCANId = 7;

        public static final int kAbsoluteEncoderChannel = 0;
        public static final double kAbsoluteEncoderZero = Units.degreesToRadians(165.7);
        public static final double kAbsoluteEncoderShift = Units.degreesToRadians(20);

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = true;

        public static final int kCurrentLimitAmps = 40;

        public static final double kMaxPositionRadians = Math.PI / 2.0; // TODO: Confirm
        public static final double kMinPositionRadians = -0.5 * Math.PI; // TODO: Confirm

        public static final double kToleranceRadians = 0.1; // TODO: Confirm

        public static class Control {
            // *** TO BE DETERMINED FROM PYTHON CODE OR PHYSICAL TUNING ***
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0;
            public static final double kG = 0;
            public static final double kV = 0;

            public static final double kMaxVelocity = 0;
            public static final double kMaxAcceleration = 0;
        }
    }

    public static class IndexConstants {
        public static final int kIndexMotorCANId = 61;
        public static final boolean kIndexMotorInverted = false;
        public static final int kIndexMotorCurrentLimit = 20;
    }
    
    public static class ShooterConstants {
        public static final int kLeftMotorCanId = 28;
        public static final int kRightMotorCanId = 8;

        public static final boolean kLeftMotorInverted = false; // TODO: Confirm
        public static final boolean kRightMotorInverted = false; // TODO: Confirm

        public static final int kCurrentLimitAmps = 40;

        public static final double kVelocityConversionFactor = (2 * Math.PI) / 60.0; // Go from RPM to radians per second TODO: Confirm
        
        public static class Control {
            // TODO: Tune or get from Python code
            public static final double kPLeft = 0;
            public static final double kILeft = 0;
            public static final double kDLeft = 0;

            public static final double kPRight = 0;
            public static final double kIRight = 0;
            public static final double kDRight = 0;
        }
    }

    // TODO: check all the constants in this class
    public static class DriveConstants {
        public static final double maxTranslationSpeedMPS = 2;
        public static final double maxRotationSpeedRadPerSec = 2;

        public static final double singleClutchTranslationFactor = 0.6;
        public static final double singleClutchRotationFactor = 0.6;
        public static final double doubleClutchTranslationFactor = 0.35;
        public static final double doubleClutchRotationFactor = 0.35;

        public static final int odometryFrequencyHz = 100;
        public static final double wheelBase = Units.inchesToMeters(27.5);
        public static final double trackWidth = Units.inchesToMeters(27.5);
        //public static final double wheelRadius = 0.0508; //Meters

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        public static class GyroConstants {
            public static final int kGyroPort = 9; // ONLY used if Gyro is a Pigeon
        }

        public static class ModuleConstants {
            public static class Common {
                public static class Drive {
                    public static final int CurrentLimit = 60;
                    public static final double gearRatio = 6.75;
                    public static final double VoltageCompensation = 12;
                    public static final double MaxModuleSpeed = 14.0; // Maximum attainable module speed
                    public static final double WheelRadius = Units.inchesToMeters(4); // Meters
                    public static final double WheelCOF = 1.0; // Coefficient of friction
                    public static final double PositionConversionFactor = 2 * WheelRadius * Math.PI / gearRatio; // Units: Meters
                    public static final double VelocityConversionFactor = PositionConversionFactor / 60; // Units: Meters per second

                    // PID constants
                    public static final double kP = 0.035;
                    public static final double kI = 0.000;
                    public static final double kD = 0.0012;

                    // Feedforward constants
                    public static final double kV = 2.78;
                    public static final double kS = 0.0;
                    public static final double kA = 0.0;
                }
    
                public static class Turn {
                    public static final int CurrentLimit = 60;
                    public static final double VoltageCompensation = 12;
                    public static final double gearRatio = 12.8;
                    public static final double PositionConversionFactor = (1 / gearRatio) * 2 * Math.PI; // Units: Radians TODO: check that radians don't break anything
                    public static final double VelocityConversionFactor = PositionConversionFactor; // Units: Radians Per Second

                    // PID constants
                    public static double kP = 0.75;
                    public static final double kI = 0.0;
                    public static final double kD = 0.0001;
                }
            }
            public static enum ModuleConfig {
    
                FrontLeft(1,11,21,-0.441162109375 +0.5),
                FrontRight(2,12,22,-0.3984375 +0.5),
                RearRight(3,13,23,-0.525146484375 ),
                RearLeft(4,14,24,-0.931396484375);
    
                public final int DrivePort;
                public final int TurnPort;
                public final int EncoderPort;
                public final double EncoderOffsetRots;
    
                private ModuleConfig(int DrivePort, int TurnPort,int EncoderPort,double EncoderOffsetRots) {
                    this.DrivePort = DrivePort;
                    this.TurnPort = TurnPort;
                    this.EncoderPort = EncoderPort;
                    this.EncoderOffsetRots = EncoderOffsetRots;
                }
            }
        }
    }

    public static class SuperstructureConstants {
        public static final double kArmMoveDebounceTimeAfterShoot = 0.2;

        // Constants for manual control TODO: tune
        public static final double kIntakeManualControlVoltage = 5;
        public static final double kArmManualControlVoltage = 4;
        public static final double kIndexManualControlVoltage = 5;
        public static final double kShooterLeftManualControlVoltage = 6;
        public static final double kShooterRightManualControlVoltage = 6;

        public enum MechanismState {
            // TODO: Tune constants, especially shooting speeds

            /** Basic state with no note */
            HOME(
                0,
                0,
                -0.6457718, 
                0, 
                0),

            /** Pick up a note from the ground */
            INTAKE(
                10, 
                4, 
                -0.6457718, 
                0, 
                0),

            /** Pre-shooting state for when at subwoofer, lowest and therefore default shooting state */
            SUBWOOFER(
                0, 
                0, 
                -0.4014257, 
                -471, 
                576),

            /** Pre-shooting state, angle just between subwoofer and mid-high */
            MID_LOW(
                0, 
                0, 
                0, 
                -400, 
                489),

            /** Pre-shooting state, angle between mid-low and amp */
            MID_HIGH(
                0, 
                0, 
                0.5, 
                -350, 
                428),
            
            /** Pre-shooting state for when at the amp, highest shooting state */
            AMP(
                0, 
                0, 
                1.3, 
                -250, 
                280),
            
            /** Shoot into the subwoofer */
            SHOOT_SUBWOOFER(
                0, 
                4, 
                SUBWOOFER.armPositionRadians, 
                SUBWOOFER.shooterLeftSpeedRadPerSec * 1.2, 
                SUBWOOFER.shooterRightSpeedRadPerSec * 1.2),
            
            /** Shoot while mid-low */
            SHOOT_MID_LOW(
                0,
                4,
                MID_LOW.armPositionRadians,
                MID_LOW.shooterLeftSpeedRadPerSec * 1.2,
                MID_LOW.shooterRightSpeedRadPerSec * 1.2
            ),

            /** Shoot while mid-high */
            SHOOT_MID_HIGH(
                0,
                4,
                MID_HIGH.armPositionRadians,
                MID_HIGH.shooterLeftSpeedRadPerSec * 1.2,
                MID_HIGH.shooterRightSpeedRadPerSec * 1.2
            ),
            
            /** Shoot into the amp */
            SHOOT_AMP(
                0,
                4,
                AMP.armPositionRadians,
                AMP.shooterLeftSpeedRadPerSec * 1.2,
                AMP.shooterRightSpeedRadPerSec * 1.2
            );

            public final double intakeVolts;
            public final double indexVolts;
            public final double armPositionRadians;
            public final double shooterLeftSpeedRadPerSec;
            public final double shooterRightSpeedRadPerSec;
            
            private MechanismState(
                double intakeVolts,
                double indexVolts,
                double armPositionRadians,
                double shooterLeftSpeedRadPerSec,
                double shooterRightSpeedRadPerSec
            ) {
                this.intakeVolts = intakeVolts;
                this.indexVolts = indexVolts;
                this.armPositionRadians = armPositionRadians;
                this.shooterLeftSpeedRadPerSec = shooterLeftSpeedRadPerSec;
                this.shooterRightSpeedRadPerSec = shooterRightSpeedRadPerSec;
            }
        }
    }
}