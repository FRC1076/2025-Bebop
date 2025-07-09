// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// The contents of this file are based upon those found in the
// AdvantageKit Skeleton Template, which was created by FRC team 6328,
// and falls under the GNU General Public License version 3,
// found in AdvantageKit-License.md in the root directory of the project

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
    */
public final class Constants {
    public static class ModeConstants {
        public static final Mode simMode = Mode.SIM;

        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
    }

    public static class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class IntakeConstants {
        public static final int kIntakeMotorCANID = 5;
        public static final boolean kIntakeMotorInverted = false;
        public static final int kIntakeMotorCurrentLimit = 20;
    }
}
