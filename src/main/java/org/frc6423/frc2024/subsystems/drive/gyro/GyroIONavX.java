package org.frc6423.frc2024.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavX implements GyroIO {

    private final AHRS navX = new AHRS();

    public GyroIONavX() {
        navX.zeroYaw();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        inputs.connected = navX.isConnected();
        inputs.gyroYaw = navX.getRotation2d();
            
    }
    
}
