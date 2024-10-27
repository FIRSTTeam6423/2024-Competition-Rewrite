package org.frc6423.frc2024.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

// ! 
public interface GyroIO {

    @AutoLog
    public static class GyroIOInputs {
        
        public boolean connected = false;
        public Rotation2d gyroYaw = new Rotation2d();
        public double gyroYawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

    }

    public default void updateInputs(GyroIOInputs inputs) {};
    
}
