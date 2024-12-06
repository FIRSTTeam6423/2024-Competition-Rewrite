// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc6423.frc2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {

    if (Robot.isReal()) {

    } else {

    }

    configureButtonBindings();

  }

  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {

    return new Command() {
      
    };
    
  }

}
