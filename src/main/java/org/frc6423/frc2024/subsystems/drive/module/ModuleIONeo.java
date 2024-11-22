package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ModuleIONeo implements ModuleIO {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax driveMotor;

    private final RelativeEncoder pivotIncrementalEncoder;
    private final DutyCycleEncoder pivotABSEncoder;
    private final RelativeEncoder driveIncrementalEncoder;

    private final PIDController pivotFeedbackController;
    private final PIDController driveFeedbackController;

    public ModuleIONeo(ModuleConfig config) {

        pivotMotor = new CANSparkMax(config.pivotMotorID(), MotorType.kBrushless);
        driveMotor = new CANSparkMax(config.driveMotorID(), MotorType.kBrushless);

        pivotABSEncoder = new DutyCycleEncoder(config.pivotABSEncoderID());
        pivotIncrementalEncoder = pivotMotor.getEncoder();
        driveIncrementalEncoder = driveMotor.getEncoder();

        pivotMotor.setSmartCurrentLimit(40);
        driveMotor.setSmartCurrentLimit(30);

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
        pivotFeedbackController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        inputs.pivotMotorEnabled = pivotMotor.getStickyFault(FaultID.kMotorFault);
        inputs.driveMotorEnabled = driveMotor.getStickyFault(FaultID.kMotorFault);

        inputs.pivotAbsolutePose = Rotation2d.fromRotations(pivotABSEncoder.getAbsolutePosition());
        inputs.pivotPose = Rotation2d.fromRotations(pivotIncrementalEncoder.getPosition());
        inputs.pivotVelRadsPerSec = pivotIncrementalEncoder.getVelocity();
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput();
        inputs.pivotOutputCurrentAmps = pivotMotor.getOutputCurrent();

        inputs.drivePoseMeters = driveIncrementalEncoder.getPosition();
        inputs.driveVelRadsPerSec = driveIncrementalEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput();
        inputs.driveOutputCurrentAmps = driveMotor.getOutputCurrent();

    }

    @Override
    public void setPivotVoltage(double voltage) {

        pivotMotor.setVoltage(voltage);

    }

    @Override
    public void setDriveVoltage(double voltage) {

        driveMotor.setVoltage(voltage);

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

    // TODO add acceleration
    @Override
    public void setDriveSetpoint(double velRadsPerSec, double accelRadsPerSecSqrd) {

        setDriveVoltage(
            driveFeedbackController.calculate(
                Units.rotationsPerMinuteToRadiansPerSecond(driveIncrementalEncoder.getVelocity()) / kDriveGearRatio, // !
                velRadsPerSec  
            )
        );

    }


    
}
