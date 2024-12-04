// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc6423.frc2024;

import static org.frc6423.frc2024.Constants.KDriveConstants.kSimConfig;

import org.frc6423.frc2024.commands.DriveCommands;
import org.frc6423.frc2024.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final Drive m_Drive;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {

    if (Robot.isReal()) {

      m_Drive = new Drive(kSimConfig);

    } else {

      m_Drive = new Drive(kSimConfig);

    }

    configureButtonBindings();

  }

  private void configureButtonBindings() {

    m_Drive.setDefaultCommand(DriveCommands.joystickDrive(
      m_Drive, 
      () -> -controller.getLeftY(), 
      () -> -controller.getLeftX(), 
      () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(m_Drive::stop, m_Drive));
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     m_Drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 m_Drive)
    //             .ignoringDisable(true));

  }

  public Command getAutonomousCommand() {

    return new Command() {
      
    };
    
  }

}
