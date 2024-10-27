// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc6423.frc2024;

import static org.frc6423.frc2024.Constants.KDriveConstants.BACKLEFT_ABS_ENCODER;
import static org.frc6423.frc2024.Constants.KDriveConstants.BACKLEFT_ABS_ENCODER_OFFSET;
import static org.frc6423.frc2024.Constants.KDriveConstants.BACKLEFT_DRIVE;
import static org.frc6423.frc2024.Constants.KDriveConstants.BACKLEFT_PIVOT;
import static org.frc6423.frc2024.Constants.KDriveConstants.BACKRIGHT_ABS_ENCODER;
import static org.frc6423.frc2024.Constants.KDriveConstants.BACKRIGHT_ABS_ENCODER_OFFSET;
import static org.frc6423.frc2024.Constants.KDriveConstants.BACKRIGHT_DRIVE;
import static org.frc6423.frc2024.Constants.KDriveConstants.BACKRIGHT_PIVOT;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTLEFT_ABS_ENCODER;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTLEFT_ABS_ENCODER_OFFSET;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTLEFT_DRIVE;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTLEFT_PIVOT;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTRIGHT_ABS_ENCODER;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTRIGHT_ABS_ENCODER_OFFSET;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTRIGHT_DRIVE;
import static org.frc6423.frc2024.Constants.KDriveConstants.FRONTRIGHT_PIVOT;

import org.frc6423.frc2024.commands.DriveCommands;
import org.frc6423.frc2024.subsystems.drive.Drive;
import org.frc6423.frc2024.subsystems.drive.gyro.GyroIO;
import org.frc6423.frc2024.subsystems.drive.gyro.GyroIONavX;
import org.frc6423.frc2024.subsystems.drive.gyro.GyroIOSim;
import org.frc6423.frc2024.subsystems.drive.module.ModuleIONeo;
import org.frc6423.frc2024.subsystems.drive.module.ModuleIOSim;
import org.frc6423.frc2024.util.ControllerUtil;
import org.frc6423.frc2024.util.ControllerUtil.IronController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.fasterxml.jackson.databind.Module;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // * ------ SUBSYSTEMS ------

  private final Drive drive;

  // * ------ COMMANDS ------
  private final CommandXboxController driveController = new CommandXboxController(0);

  // * ------ AUTO (womp womp) ------
  private final SendableChooser<Command> autoChooser;

  // * ------ CONTROLLERS ------
  public RobotContainer() {

    if (Robot.isReal()) {
      drive = new Drive(
        new GyroIONavX() {}, 
        new ModuleIONeo(0), 
        new ModuleIONeo(1), 
        new ModuleIONeo(2), 
        new ModuleIONeo(3)
      );
    } else {
      drive = new Drive(
        new GyroIOSim() {}, 
        new ModuleIOSim() {}, 
        new ModuleIOSim()  {}, 
        new ModuleIOSim() {}, 
        new ModuleIOSim() {}
      );
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive,
        () -> driveController.getLeftY(), 
        () -> driveController.getLeftX(), 
        () -> driveController.getRightX()
        )
    );
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
