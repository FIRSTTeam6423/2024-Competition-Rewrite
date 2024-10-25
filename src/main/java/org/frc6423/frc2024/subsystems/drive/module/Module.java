package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveA;
import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveS;
import static org.frc6423.frc2024.Constants.KDriveConstants.kDriveV;
import static org.frc6423.frc2024.Constants.KDriveConstants.kModuleDriveD;
import static org.frc6423.frc2024.Constants.KDriveConstants.kModuleDriveI;
import static org.frc6423.frc2024.Constants.KDriveConstants.kModuleDriveP;
import static org.frc6423.frc2024.Constants.KDriveConstants.kModulePivotD;
import static org.frc6423.frc2024.Constants.KDriveConstants.kModulePivotI;
import static org.frc6423.frc2024.Constants.KDriveConstants.kModulePivotP;

import org.frc6423.frc2024.Constants;
import org.frc6423.frc2024.Robot;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int ID;

    private final PIDController pivotPIDController;
    private final PIDController drivePIDController;
    private final SimpleMotorFeedforward driveFeedforward;
    private Rotation2d pivotSetpoint = null;
    private Double driveSetpoint = null;
    private Rotation2d pivotRelativeOffset = null;

    public Module(ModuleIO io, int ID) {
        this.ID = ID;
        this.io = io;

        pivotPIDController = new PIDController(kModulePivotP,kModulePivotI, kModulePivotD);
        drivePIDController = new PIDController(kModuleDriveP, kModuleDriveI, kModuleDriveD);
        driveFeedforward = new SimpleMotorFeedforward(kDriveS, kDriveV, kDriveA);

        pivotPIDController.enableContinuousInput(-Math.PI, Math.PI);
        io.setDriveBreakMode(IdleMode.kBrake);
        io.setPivotBreakMode(IdleMode.kBrake);

    }

    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(ID), inputs);

        if (pivotRelativeOffset == null && inputs.pivotAbsolutePose.getRadians() != 0.0) {
            pivotRelativeOffset = inputs.pivotAbsolutePose.minus(inputs.pivotPosition);
        }

        if (pivotSetpoint != null) {
            io.setPivotVoltage(
                pivotPIDController.calculate(getAngle().getRadians(), pivotSetpoint.getRadians())
            );
        }

        if (driveSetpoint != null) {
            // Scale velocity based on turn error
            //
            // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
            // towards the setpoint, its velocity should increase. This is achieved by
            // taking the component of the velocity in the direction of the setpoint.
            double adjustSpeedSetpoint = driveSetpoint * Math.cos(pivotPIDController.getPositionError());

            // Run drive controller
            double velocityRadPerSec = adjustSpeedSetpoint / Units.inchesToMeters(2.0);
            io.setDriveVoltage(
                driveFeedforward.calculate(velocityRadPerSec)
                    + drivePIDController.calculate(inputs.driveVelRadPerSec, velocityRadPerSec));
        }

    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        pivotSetpoint = optimizedState.angle;
        driveSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;

    }

    public void runCharacterization(double volts) {
        pivotSetpoint = new Rotation2d();

        io.setDriveVoltage(volts);
        driveSetpoint = null;
    }

    public void stop() {
        
        io.setPivotVoltage(0.0);
        io.setDriveVoltage(0.0);

        pivotSetpoint = null;
        driveSetpoint = null;

    }

    public Rotation2d getAngle() {
        if (pivotRelativeOffset == null) {
        return new Rotation2d();
        } else {
        return inputs.pivotPosition.plus(pivotRelativeOffset);
        }
    }

    public double getPositionMeters() {
        return inputs.drivePoseRad.getRadians() * Units.inchesToMeters(2);
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelRadPerSec * Units.inchesToMeters(2);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelRadPerSec;
    }
    
}
