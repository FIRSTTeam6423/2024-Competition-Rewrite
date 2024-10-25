package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveGearRatio;
import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveMotorCurrentLimit;
import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveReduction;
import static org.frc6423.frc2024.Constants.KDriveConstants.kPivotGearRatio;
import static org.frc6423.frc2024.Constants.KDriveConstants.kPivotMotorCurrentLimit;

import java.util.OptionalDouble;
import java.util.Queue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ModuleIONeo implements ModuleIO {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax driveMotor;

    private final RelativeEncoder pivotRelativeEncoder;
    private final RelativeEncoder driveRelativeEncoder;
    private final DutyCycleEncoder pivotEncoder; // !
    
    // private final Queue<Double> pivotPoseQueue;
    // private final Queue<Double> drivePoseQueue;

    private final Rotation2d pivotOffset;

    public ModuleIONeo(int pivotMotorID, int driveMotorID, int pivotEncoderID, Rotation2d pivotOffset, boolean pivotInverted, boolean driveInverted) {

        this.pivotOffset = pivotOffset;

        pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        // pivotMotor.setInverted(pivotInverted);
        // pivotMotor.restoreFactoryDefaults();
        // pivotMotor.setIdleMode(IdleMode.kBrake);
        // pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit); // !

        // pivotEncoder.reset();
        // this.pivotOffset = pivotOffset;

        // driveMotor.setInverted(driveInverted);
        // driveMotor.restoreFactoryDefaults();
        // driveMotor.setIdleMode(IdleMode.kBrake);
        // driveMotor.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        // driveMotor.burnFlash();

        // driveRelativeEncoder.setPosition(0);
        // driveRelativeEncoder.setPositionConversionFactor(DRIVE_ROTATIONS_TO_METERS);
        // driveRelativeEncoder.setVelocityConversionFactor(RPM_TO_METERS_PER_SEC);
        // driveRelativeEncoder.setMeasurementPeriod(1); // 32

        pivotMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        pivotRelativeEncoder = pivotMotor.getEncoder();
        driveRelativeEncoder = driveMotor.getEncoder();

        pivotMotor.setInverted(pivotInverted);
        driveMotor.setInverted(driveInverted);
        pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
        driveMotor.setSmartCurrentLimit(kDriveMotorCurrentLimit);
        pivotMotor.enableVoltageCompensation(12.0); // !
        driveMotor.enableVoltageCompensation(12.0);

        pivotRelativeEncoder.setPosition(0.0);
        pivotRelativeEncoder.setMeasurementPeriod(10); // !
        pivotRelativeEncoder.setAverageDepth(2);

        driveRelativeEncoder.setPosition(0.0);
        driveRelativeEncoder.setMeasurementPeriod(10);
        driveRelativeEncoder.setAverageDepth(2);

        pivotEncoder = new DutyCycleEncoder(pivotEncoderID);

        // pivotPoseQueue = 
        //     SparkMaxOdometryThread.getInstance()
        //         .registerSignal(
        //             () -> {
        //                 double value = pivotRelativeEncoder.getPosition();
        //                 if (pivotMotor.getLastError() == REVLibError.kOk) {
        //                     return OptionalDouble.of(value);
        //                 } else {
        //                     return OptionalDouble.empty();
        //                 }
        //             }
        //         );
        // drivePoseQueue =
        // SparkMaxOdometryThread.getInstance()
        //     .registerSignal(
        //         () -> {
        //           double value = driveRelativeEncoder.getPosition();
        //           if (driveMotor.getLastError() == REVLibError.kOk) {
        //             return OptionalDouble.of(value);
        //           } else {
        //             return OptionalDouble.empty();
        //           }
        //         });

        pivotMotor.burnFlash();
        driveMotor.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        inputs.pivotAbsolutePose = new Rotation2d(pivotEncoder.getAbsolutePosition()).minus(pivotOffset); // !
        inputs.pivotPosition = Rotation2d.fromRotations(kPivotMotorCurrentLimit / kPivotGearRatio);
        inputs.pivotVelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotRelativeEncoder.getVelocity()) / kPivotGearRatio;
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    
        inputs.drivePoseRad = Rotation2d.fromRotations(Units.rotationsToRadians(driveRelativeEncoder.getPosition()) / kDriveGearRatio);
        inputs.driveVelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveRelativeEncoder.getVelocity()) / kDriveGearRatio;
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        
        // inputs.odometryDrivePositionsMeters =
        //     drivePoseQueue.stream()
        //         .mapToDouble(
        //             motorPoseRevs -> 
        //                 Units.rotationsToRadians(motorPoseRevs / kDriveReduction)
        //                     * 0.0508 // ! inches to meters
        //         ).toArray();
        // inputs.odometryTurnPositions =
        //     pivotPoseQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
        
        // drivePoseQueue.clear();
        // pivotPoseQueue.clear();
    }

    @Override
    public void setPivotVoltage(double volts) {
    
        pivotMotor.setVoltage(volts);
        
    }

    @Override
    public void setDriveVoltage(double volts) {
        
        driveMotor.setVoltage(volts);
        
    }

    @Override
    public void setPivotBreakMode(IdleMode mode) {

        pivotMotor.setIdleMode(mode);

    }

    @Override
    public void setDriveBreakMode(IdleMode mode) {

        driveMotor.setIdleMode(mode);

    }
    
}
