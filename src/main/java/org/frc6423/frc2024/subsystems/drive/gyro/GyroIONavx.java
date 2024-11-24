package org.frc6423.frc2024.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavx implements GyroIO {

    private final AHRS navx = new AHRS();

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        inputs.gyroEnabled = true;

        inputs.yawPose = navx.getRotation2d();
        inputs.yawVelRadsPerSec = navx.getVelocityZ(); // ! 

    }
    
}
