package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveGearRatio;
import static org.frc6423.frc2024.Constants.KDriveConstants.kPivotGearRatio;

import org.frc6423.frc2024.Constants.KDriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

public class ModuleIOSim implements ModuleIO {

    private final DCMotorSim pivotMotor;
    private final DCMotorSim driveMotor;

    private final DutyCycleEncoderSim pivotABSEncoder;

    private final PIDController pivotFeedbackController;
    private final PIDController driveFeedbackController;

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

        pivotABSEncoder = new DutyCycleEncoderSim(config.pivotABSEncoderID());


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



    }

    @Override
    public void setPivotVoltage(double voltage) {

        pivotMotor.setInputVoltage(voltage);

    }

    @Override
    public void setDriveVoltage(double voltage) {

        driveMotor.setInputVoltage(voltage);

    }

    @Override
    public void setPivotSetpoint(Rotation2d angle) {

        setPivotVoltage(
            pivotFeedbackController.calculate(
                pivotABSEncoder.getAbsolutePosition(),
                angle.getRotations() // ! Check units
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
