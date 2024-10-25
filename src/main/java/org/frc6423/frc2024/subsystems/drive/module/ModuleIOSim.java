package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO{

    private final DCMotorSim pivotSim =
        new DCMotorSim(DCMotor.getNEO(1), kPivotReduction, 0.025);
    private final DCMotorSim driveSim =
        new DCMotorSim(DCMotor.getNEO(1), kDriveReduction, 0.004);

    private final PIDController pivotPIDController = new PIDController(0.0, 0.0, 0.0, 0.02);
    private final PIDController drivePIDController = new PIDController(0.0, 0.0, 0.0, 0.02);

    private double driveAppliedVolts, pivotAppliedVolts;
    
    public ModuleIOSim() {
        pivotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        if (DriverStation.isDisabled()) {
            stop();
        }

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

    public void stop() {
        setDriveVoltage(0);
        setPivotVoltage(0);
    }
    
}
