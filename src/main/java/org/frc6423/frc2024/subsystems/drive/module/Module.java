package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import org.frc6423.frc2024.Constants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {

    private final int id;
    private final String akitLoggingPath;
    
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs;

    private SwerveModuleState m_ModuleState;
    private SwerveModulePosition m_ModulePosition;

    private final PIDController m_FeedbackPivot;
    private final PIDController m_FeedbackDrive;
    private final SimpleMotorFeedforward m_FeedforwardDrive;

    private Rotation2d m_PivotSetpoint;
    private Double m_DriveSetpoint;

    public Module(ModuleIO io, int id) {

        this.id = id;
        akitLoggingPath = "Drive/Module" + id;

        this.io = io;
        inputs = new ModuleIOInputsAutoLogged();


        switch (Constants.getRobotMode()) {
            case REAL:
                m_FeedbackPivot = new PIDController(
                    kModulePivotP, 
                    kModulePivotI, 
                    kModulePivotD
                );

                m_FeedbackDrive = new PIDController(
                    kModuleDriveP, 
                    kModuleDriveI, 
                    kModuleDriveD
                );

                m_FeedforwardDrive = new SimpleMotorFeedforward(kDriveS, kDriveV);
                break;
            case REPLAY:
                m_FeedbackPivot = new PIDController(
                    kModulePivotP, 
                    kModulePivotI, 
                    kModulePivotD
                );

                m_FeedbackDrive = new PIDController(
                    kModuleDriveP, 
                    kModuleDriveI, 
                    kModuleDriveD
                );

                m_FeedforwardDrive = new SimpleMotorFeedforward(kDriveS, kDriveV);
                break;
            case SIMULATED:
                m_FeedbackPivot = new PIDController(
                    kModulePivotP, 
                    kModulePivotI, 
                    kModulePivotD
                );

                m_FeedbackDrive = new PIDController(
                    kModuleDriveP, 
                    kModuleDriveI, 
                    kModuleDriveD
                );

                m_FeedforwardDrive = new SimpleMotorFeedforward(kDriveS, kDriveV);
                break;
            default:
                m_FeedbackPivot = new PIDController(
                    kModulePivotP, 
                    kModulePivotI, 
                    kModulePivotD
                );

                m_FeedbackDrive = new PIDController(
                    kModuleDriveP, 
                    kModuleDriveI, 
                    kModuleDriveD
                );

                m_FeedforwardDrive = new SimpleMotorFeedforward(kDriveS, kDriveV);
                break;
        }
        m_FeedbackPivot.enableContinuousInput(Math.PI, -Math.PI);

        m_ModuleState = new SwerveModuleState();
        m_ModulePosition = new SwerveModulePosition();

        Logger.recordOutput(akitLoggingPath + "/ModuleState", m_ModuleState);

    }

    /**
     * womp womp bruv
     */
    public void periodic() {

        io.updateInputs(inputs);

        if (m_PivotSetpoint != null) {

            io.setPivotVoltage(
                m_FeedbackPivot.calculate(getModuleState().angle.getRadians(), m_PivotSetpoint.getRadians())
            );

            if (m_DriveSetpoint != null) {

                double velRadsPerSec = (m_DriveSetpoint * Math.cos(m_FeedbackPivot.getPositionError()))/kWheelRadius;
                io.setDriveVoltage(
                    m_FeedforwardDrive.calculate(velRadsPerSec)
                    + m_FeedbackDrive.calculate(inputs.driveVelRadPerSec, velRadsPerSec)
                );

            }

        }

    }

    public SwerveModuleState setModuleSetpoints(SwerveModuleState state) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getModuleState().angle);

        m_PivotSetpoint = optimizedState.angle;
        m_DriveSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;

    }

    /**
     * Disables output to motors and disables close loop control
     */
    public void stop() {

        io.setPivotVoltage(0);
        io.setDriveVoltage(0);

        m_PivotSetpoint = null;
        m_DriveSetpoint = null;

    }

    public void setIdleMode(IdleMode idleMode) {

        io.setIdleMode(idleMode, idleMode);

    }

    public Rotation2d getPivotAngle() {

        return inputs.pivotABSPose;

    }

    public double getVelocityMetersPerSec() {

        return inputs.driveVelRadPerSec * kWheelRadius;

    }

    public double getCharacterizationVelocity() {

        return inputs.driveVelRadPerSec;
        
    }

    public double getPositionMeters() {

        return inputs.drivePoseRads * kWheelRadius;

    }

    public SwerveModuleState getModuleState() {

        return new SwerveModuleState(getVelocityMetersPerSec(), getPivotAngle());

    }

    public SwerveModulePosition getModulePosition() {

        return new SwerveModulePosition(getVelocityMetersPerSec(), getPivotAngle());

    }

}
