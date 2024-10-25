package org.frc6423.frc2024.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {

    private Rotation2d yaw = new Rotation2d();

    @Override
    public void updateInputs(GyroIOInputs inputs) {
    
        inputs.connected = true;
        inputs.gyroYaw = yaw;
    
    }

    public void resetSimulationYaw() {
        yaw = new Rotation2d();
    }

    public void setSimulationYaw(Rotation2d set) {
        yaw = set;
    }
    
}
