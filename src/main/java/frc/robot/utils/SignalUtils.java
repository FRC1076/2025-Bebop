// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;

public class SignalUtils {
    /** Returns a spark max error signal. WARNING: NOT THREAD-SAFE, LOCKING MUST BE IMPLEMENTED IN THE CALLING THREAD TO PREVENT RACE CONDITIONS*/
    public static BooleanSupplier getSparkMaxErrorSignal(SparkMax motor){
        return () -> (motor.getLastError() != REVLibError.kOk);
    }

    /** Returns a Spark Max position signal. WARNING: NOT THREAD-SAFE */
    public static DoubleSupplier getSparkMaxPositionSignal(SparkMax motor){
        return () -> (motor.getEncoder().getPosition());
    }
}