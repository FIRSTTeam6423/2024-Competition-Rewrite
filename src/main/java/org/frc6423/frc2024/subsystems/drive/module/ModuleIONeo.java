package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ModuleIONeo implements ModuleIO {

    private final ModuleConfig m_ModuleConfig;

    private final CANSparkMax m_PivotMotor;
    private final CANSparkMax m_DriveMotor;
    
    private final DutyCycleEncoder m_PivotABSEncoder;
    private final RelativeEncoder m_PivotRelativeEncoder;
    private final RelativeEncoder m_DriveRelativeEncoder;

    private final PIDController m_FeedbackPivotMotor;
    private final PIDController m_FeedbackDriveMotor;
    private final SimpleMotorFeedforward m_FeedforwardDriveMotor;

    public ModuleIONeo(ModuleConfig config) {
        
        m_PivotMotor = new CANSparkMax(config.pivotMotorID(), MotorType.kBrushless);
        m_DriveMotor = new CANSparkMax(config.driveMotorID(), MotorType.kBrushless);

        m_PivotABSEncoder = new DutyCycleEncoder(config.pivotABSEncoderID());
        m_PivotRelativeEncoder = m_PivotMotor.getEncoder();
        m_DriveRelativeEncoder = m_DriveMotor.getEncoder();


        m_FeedbackPivotMotor = new PIDController(
            kModulePivotP, 
            kModulePivotI, 
            kModulePivotD
        );
        
        m_FeedbackDriveMotor = new PIDController(
            kModuleDriveP, 
            kModuleDriveI, 
            kModuleDriveD
        );

        m_FeedforwardDriveMotor = new SimpleMotorFeedforward(
            0, 
            0.2
        );


        m_ModuleConfig = config;
        m_PivotABSEncoder.setPositionOffset(config.pivotABSOffset().getDegrees());

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        // DutyCycleEncoder returns in Rotations
        inputs.pivotABSPose = Rotation2d.fromRotations(m_PivotABSEncoder.getAbsolutePosition()); 
        inputs.pivotPose = Rotation2d.fromRotations(m_PivotRelativeEncoder.getPosition());
        inputs.pivotVelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_PivotRelativeEncoder.getVelocity());
        inputs.pivotAppliedVolts = m_PivotMotor.getAppliedOutput() * m_PivotMotor.getBusVoltage();
        inputs.pivotCurrentAmps = m_PivotMotor.getOutputCurrent();

        inputs.drivePoseRads = Units.rotationsToRadians(m_DriveRelativeEncoder.getPosition());
        inputs.driveVelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_DriveRelativeEncoder.getVelocity());
        inputs.driveAppliedVolts = m_DriveMotor.getAppliedOutput() * m_DriveMotor.getBusVoltage();
        inputs.driveCurrentAmps = m_DriveMotor.getOutputCurrent();

    }

    @Override
    public void setPivotVoltage(double voltage) {
    }

    @Override
    public void setDriveVoltage(double voltage) {
    }

    @Override
    public void setIdleMode(IdleMode pivotMode, IdleMode driveMode) {
    }
    
}
