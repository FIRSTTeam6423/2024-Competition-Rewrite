package org.frc6423.frc2024.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    @AutoLog
    public static class GyroIOInputs {

        public boolean enabled = false;
        public Rotation2d yaw = new Rotation2d();
        public double yawVelocityRadsPerSec = 0.0;

    }

    public void updateInputs(GyroIOInputs inputs);

}