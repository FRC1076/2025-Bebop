// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// The contents of this file are based off those of
// team 6328 Mechanical Advantage's Spark Swerve Template,
// whose license can be found in AdvantageKit-License.md
// in the root directory of this file

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs{
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnAbsolutePositionRadians = turnAbsolutePosition.getRadians();
        public double turnVelocityRadiansPerSecond = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public abstract void updateInputs(ModuleIOInputs inputs);

    public default void setDriveVolts(double volts) {}

    public default void setTurnVolts(double volts) {}

    public default void setDriveVelocity(double velocityMetersPerSec, double FFVolts) {}

    public default void setTurnPosition(double positionRadians, double FFVolts) {}
}