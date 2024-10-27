package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO{

    private final DCMotorSim pivotSim =
        new DCMotorSim(DCMotor.getNEO(1), kPivotReduction, 0.025);
    private final DCMotorSim driveSim =
        new DCMotorSim(DCMotor.getNEO(1), kDriveReduction, 0.004);

    private double driveAppliedVolts, pivotAppliedVolts;

    private Rotation2d intPivot = new Rotation2d();
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        driveSim.update(0.02);
        pivotSim.update(0.02);

        inputs.pivotAbsolutePose = new Rotation2d(pivotSim.getAngularPositionRad()).plus(this.intPivot);
        inputs.pivotPosition = new Rotation2d(pivotSim.getAngularPositionRad());
        inputs.pivotVelRadPerSec = pivotSim.getAngularVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;

        inputs.drivePoseRad = driveSim.getAngularPositionRad();
        inputs.driveVelRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = this.driveAppliedVolts;

    }
    @Override
    public void setPivotVoltage(double volts) {
        
        pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        pivotSim.setInputVoltage(pivotAppliedVolts);

    }
    @Override
    public void setDriveVoltage(double volts) {
        
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);

    }

    @Override
    public void setPivotBreakMode(IdleMode mode) {

    }

    @Override
    public void setDriveBreakMode(IdleMode mode) {

    }

}
