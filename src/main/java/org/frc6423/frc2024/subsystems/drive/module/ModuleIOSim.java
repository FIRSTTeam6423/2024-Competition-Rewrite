package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveGearRatio;
import static org.frc6423.frc2024.Constants.KDriveConstants.kPivotGearRatio;

import org.frc6423.frc2024.Constants.KDriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {

    private final DCMotorSim pivotMotor;
    private final DCMotorSim driveMotor;

    private final PIDController pivotFeedbackController;
    private final PIDController driveFeedbackController;

    private double pivotAppliedVolts = 0.0;
    private double driveAppliedVolts = 0.0;

    public ModuleIOSim(ModuleConfig config) {

        pivotMotor = new DCMotorSim(
            DCMotor.getNEO(config.pivotMotorID()), 
            kPivotGearRatio, 
            0.025
        );
        driveMotor = new DCMotorSim(
            DCMotor.getNEO(config.driveMotorID()), 
            kDriveGearRatio, 
            0.004
        );

        

        pivotFeedbackController = new PIDController(
            0, 
            0, 
            0
        );
        driveFeedbackController = new PIDController(
            0, 
            0, 
            0
        );

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        inputs.pivotMotorEnabled = true;
        inputs.driveMotorEnabled = true;

        inputs.pivotAbsolutePose = Rotation2d.fromRadians(pivotMotor.getAngularPositionRotations());
        inputs.pivotPose = Rotation2d.fromRadians(pivotMotor.getAngularPositionRad());
        inputs.pivotVelRadsPerSec = pivotMotor.getAngularVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
        inputs.driveAppliedVolts = driveAppliedVolts;

    }

    @Override
    public void setPivotVoltage(double voltage) {

        pivotAppliedVolts = MathUtil.clamp(voltage, -12, 12);
        pivotMotor.setInputVoltage(voltage);

    }

    @Override
    public void setDriveVoltage(double voltage) {

        driveAppliedVolts = MathUtil.clamp(voltage, -12, 12);
        driveMotor.setInputVoltage(voltage);

    }

    @Override
    public void setPivotSetpoint(Rotation2d angle) {

        setPivotVoltage(
            pivotFeedbackController.calculate(
                pivotMotor.getAngularPositionRotations(),
                angle.getRotations()
            )
        );

    }

    @Override
    public void setDriveSetpoint(double velRadsPerSec, double accelRadsPerSecSqrd) {

        setDriveVoltage(
            driveFeedbackController.calculate(
                Units.rotationsPerMinuteToRadiansPerSecond(driveMotor.getAngularVelocityRadPerSec()) / kDriveGearRatio, // !
                velRadsPerSec  
            )
        );

    }
    
}
