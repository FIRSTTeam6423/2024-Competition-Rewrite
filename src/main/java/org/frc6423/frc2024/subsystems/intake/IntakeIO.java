package org.frc6423.frc2024.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {

        public Rotation2d pivotAngle = new Rotation2d();
        public double pivotAppliedVoltage = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double rollerAppliedVoltage = 0.0;
        public double rollerCurrentAmps = 0.0;

        public boolean intakeButtonPressed = false;

    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setPivotMotorSpeed(double speed);

    public void setPivotMotorVoltage(double voltage);

    public void setRollerMotorSpeed(double speed);

    public void setRollerMotorVoltage(double voltage);

}
