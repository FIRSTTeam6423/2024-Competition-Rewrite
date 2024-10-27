package org.frc6423.frc2024.subsystems.drive.module;

import static edu.wpi.first.units.Units.Rotation;
import static org.frc6423.frc2024.Constants.KDriveConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class ModuleIOSim implements ModuleIO{

    // private final DCMotorSim pivotSim =
    //     new DCMotorSim(DCMotor.getNEO(1), kPivotReduction, 0.025);
    // private final DCMotorSim driveSim =
    //     new DCMotorSim(DCMotor.getNEO(1), kDriveReduction, 0.004);
    private final LinearSystemSim<N2, N1, N2> driveSim, pivotSim;

    private double driveAppliedVolts, pivotAppliedVolts;

    private Rotation2d intPivot = new Rotation2d();

    public ModuleIOSim() {
        driveSim = 
        new LinearSystemSim<N2, N1, N2>(
            new LinearSystem<N2, N1, N2>(
                MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -kDriveA / kDriveA),
                MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / kDriveA),
                MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
                MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0)));

        pivotSim =
            new LinearSystemSim<N2, N1, N2>(
                new LinearSystem<N2, N1, N2>(
                    MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -kDriveV / kDriveA),
                    MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / kDriveA),
                    MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
                    MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0)));
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        driveSim.update(0.02);
        pivotSim.update(0.02);

        inputs.pivotAbsolutePose = Rotation2d.fromRadians(pivotSim.getOutput(0));
        inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getOutput(0));
        inputs.pivotVelRadPerSec = pivotSim.getOutput(1);
        
        inputs.drivePoseRad = driveSim.getOutput(0);
        inputs.driveVelRadPerSec = driveSim.getOutput(1);

        // inputs.pivotAbsolutePose = new Rotation2d(pivotSim.getAngularPositionRad()).plus(this.intPivot);
        // inputs.pivotPosition = new Rotation2d(pivotSim.getAngularPositionRad());
        // inputs.pivotVelRadPerSec = pivotSim.getAngularVelocityRadPerSec();
        // inputs.pivotAppliedVolts = pivotAppliedVolts;

        // inputs.drivePoseRad = driveSim.getAngularPositionRad();
        // inputs.driveVelRadPerSec = driveSim.getAngularVelocityRadPerSec();
        // inputs.driveAppliedVolts = this.driveAppliedVolts;

    }
    @Override
    public void setPivotVoltage(double volts) {
        
        pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        pivotSim.setInput(pivotAppliedVolts);

    }
    @Override
    public void setDriveVoltage(double volts) {
        
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInput(driveAppliedVolts);

    }

    @Override
    public void setPivotBreakMode(IdleMode mode) {

    }

    @Override
    public void setDriveBreakMode(IdleMode mode) {

    }

}
