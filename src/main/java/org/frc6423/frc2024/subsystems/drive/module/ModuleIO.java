package org.frc6423.frc2024.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {

        public static Rotation2d pivotAbsolutePose = new Rotation2d();
        public static Rotation2d pivotPosition = new Rotation2d();
        public static double pivotVelRadPerSec = 0.0;
        public static double pivotAppliedVolts = 0.0;
        
        public static double drivePoseRad = 0.0;
        public static double driveVelRadPerSec = 0.0;
        public static double driveAppliedVolts = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setPivotVoltage(double volts);

    public void setDriveVoltage(double volts);

    public void setPivotBreakMode(IdleMode mode);

    public void setDriveBreakMode(IdleMode mode);
    
}
