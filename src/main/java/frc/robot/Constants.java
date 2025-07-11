// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
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
        public static final boolean kFollowMotorInverted = false;

        public static final int kCurrentLimitAmps = 40;

        public static final double kMaxPositionRadians = Math.PI / 2.0;
        public static final double kMinPositionRadians = -0.5 * Math.PI; // *** TO BE DETERMINED ***

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
}
