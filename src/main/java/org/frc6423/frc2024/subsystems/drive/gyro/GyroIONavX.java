package org.frc6423.frc2024.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavX implements GyroIO {

    private final AHRS gyro;

    public GyroIONavX() {

        gyro = new AHRS();
        gyro.reset();

    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        inputs.enabled = true;
        inputs.yaw = Rotation2d.fromDegrees(gyro.getYaw());
        inputs.yawVelocityRadsPerSec = 0.0;

    }
    
}
