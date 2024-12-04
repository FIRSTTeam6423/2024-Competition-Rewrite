package org.frc6423.frc2024.subsystems.drive.module;

import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import org.frc6423.frc2024.Constants.KDriveConstants.ModuleConfig;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {

    private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 150, 0.004);

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private final PIDController m_FeedbackPivotMotor;
    private final PIDController m_FeedbackDriveMotor;
    private final SimpleMotorFeedforward m_FeedforwardDriveMotor;

    public ModuleIOSim(ModuleConfig config) {

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

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        driveSim.update(0.02);
        turnSim.update(0.02);

        inputs.drivePoseRads = driveSim.getAngularPositionRad();
        inputs.driveVelRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.pivotABSPose =
            new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.pivotPose = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.pivotVelRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.pivotAppliedVolts = turnAppliedVolts;
        inputs.pivotCurrentAmps = turnSim.getCurrentDrawAmps();

    }

    @Override
    public void setPivotVoltage(double voltage) {

        turnAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);

    }

    @Override
    public void setDriveVoltage(double voltage) {

        driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
        
    }

    @Override
    public void setIdleMode(IdleMode pivotMode, IdleMode driveMode) {
    }
    
}
