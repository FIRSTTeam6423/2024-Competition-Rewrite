package org.frc6423.frc2024.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {

        public boolean gyroEnabled = false;

        public Rotation2d yawPose = new Rotation2d();
        public double yawVelRadsPerSec = 0.0;

    }

    void updateInputs(GyroIOInputs inputs);
    
}
