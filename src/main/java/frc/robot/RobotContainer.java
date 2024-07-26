// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.NoteElevator;

public class RobotContainer {
  CommandXboxController commandXboxController = new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);

  Climber climber = new Climber("src/main/deploy/configs/climber/climberConf.json");
  NoteElevator noteElevator = new NoteElevator("src/main/deploy/configs/notevator/notevatorConf.json");

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // commandXboxController.y().onTrue(climber.);
    // commandXboxController.x().onTrue(climber.);
    
    // commandXboxController.y().onTrue(climber.)
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
