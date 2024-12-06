package org.frc6423.frc2024.subsystems.intake;

import static org.frc6423.frc2024.Constants.KIntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIONeo implements IntakeIO {

    // initalize motors
    private final CANSparkMax m_PivotMotor;
    private final CANSparkMax m_RollerMotor;

    // initalize the absolute encoder that we'll use for getting intake angles
    private final DutyCycleEncoder m_PivotABSEncoder;

    // initalize the button in the intake for detecting notes
    private final DigitalOutput m_IntakeLimitSwitch;

    public IntakeIONeo() {

        // initalize motors
        m_PivotMotor = new CANSparkMax(kPivotMotorID, MotorType.kBrushless);
        m_RollerMotor = new CANSparkMax(kRollerMotorID, MotorType.kBrushless);

        // initalize the absolute encoder that we'll use for getting intake angles
        m_PivotABSEncoder = new DutyCycleEncoder(kPivotABSEncoder);

        // initalize the button in the intake for detecting notes
        m_IntakeLimitSwitch = new DigitalOutput(kIntakeLimitSwitchID);

    }

    // Logs hardware inputs every loop. For us, that's every 0.02 seconds
    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        // Intake motor inputs
        inputs.pivotAngle = Rotation2d.fromDegrees(m_PivotABSEncoder.getAbsolutePosition());
        inputs.pivotAppliedVoltage = m_PivotMotor.getAppliedOutput() * m_PivotMotor.getBusVoltage();
        inputs.pivotCurrentAmps = m_PivotMotor.getOutputCurrent();

        // Roller motor inputs
        inputs.rollerAppliedVoltage = m_RollerMotor.getAppliedOutput() * m_PivotMotor.getBusVoltage();
        inputs.rollerCurrentAmps = m_RollerMotor.getOutputCurrent();

        // Returns true when the button is pressed
        inputs.intakeButtonPressed = m_IntakeLimitSwitch.get();

    }

    /**
     * Sets Pivot Motor speed to a value between -1.0 to 1.0
     */
    @Override
    public void setPivotMotorSpeed(double speed) {

        m_PivotMotor.set(speed);

    }

    /**
     * Sets Pivot motor voltage
     */
    @Override
    public void setPivotMotorVoltage(double voltage) {

        m_PivotMotor.setVoltage(voltage);
    
    }

    /**
     * Sets Roller Motor speed to a value between -1.0 to 1.0
     */
    @Override
    public void setRollerMotorSpeed(double speed) {

        m_RollerMotor.set(speed);

    }

    /**
     * Sets Roller Motor voltage
     */
    @Override
    public void setRollerMotorVoltage(double voltage) {

        m_RollerMotor.setVoltage(voltage);
            
    }
    
}
