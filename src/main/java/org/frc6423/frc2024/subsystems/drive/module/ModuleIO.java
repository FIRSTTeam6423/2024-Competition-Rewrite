package org.frc6423.frc2024.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    @AutoLog
    public class ModuleIOInputs {

        public boolean pivotEnabled = false;
        public boolean driveEnabled = false;

        public Rotation2d pivotABSPose = new Rotation2d();
        public Rotation2d pivotPose = new Rotation2d();
        public double pivotVelRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double drivePoseRads = 0.0;
        public double driveVelRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setPivotVoltage(double voltage);

    public void setDriveVoltage(double voltage);

    public void setIdleMode(IdleMode pivotMode, IdleMode driveMode);
    
}
