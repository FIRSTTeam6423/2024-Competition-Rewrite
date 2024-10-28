// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc6423.frc2024;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  
  private PowerDistribution PDH = Robot.getPDH();

  // * ------ SUBSYSTEMS ------
  // * ------ COMMANDS ------
  // * ------ AUTO ------

  private final LoggedDashboardChooser<Command> autoChooser;

  // * ------ Container ------
  public RobotContainer() {

    if (Robot.isReal()) {} else {}

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();

  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {

    return autoChooser.get();
    
  }

}
