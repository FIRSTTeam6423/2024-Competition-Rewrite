package org.frc6423.frc2024.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

    @AutoLog
    class ModuleIOInputs {

        public boolean pivotMotorEnabled = false;
        public boolean driveMotorEnabled = false;

        public Rotation2d pivotAbsolutePose = new Rotation2d();
        public Rotation2d pivotPose = new Rotation2d();
        public double pivotVelRadsPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotOutputCurrentAmps = 0.0;

        public double drivePoseMeters = 0.0;
        public double driveVelRadsPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveOutputCurrentAmps = 0.0;


    }

    void updateInputs(ModuleIOInputs inputs);

    /**
     * Set Pivot Motor voltage
     * 
     * @param voltage {@link Double}
     */
    void setPivotVoltage(double voltage);

    /**
     * Set Drive Motor voltage
     * 
     * @param voltage {@link Double}
     */
    void setDriveVoltage(double voltage);    

    /**
     * Run Pivot Motor to a specified angle
     * 
     * @param angle {@link Rotation2d} Target Angle
     */
    void setPivotSetpoint(Rotation2d angle);

    /**
     * Accelerate Drive Motor to specified speed at specified acceleration
     * 
     * @param velRadsPerSec {@link Double} Target Velocity
     * @param accelRadsPerSecSqrd {@link Double} Acceleration
     */
    void setDriveSetpoint(double velRadsPerSec, double accelRadsPerSecSqrd);
 
}
