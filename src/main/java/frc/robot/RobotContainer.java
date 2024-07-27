// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem("configs/indexer/indexerConf.json");
  Shooter shooterSubsystem = new Shooter("configs/shooter/shooterConf");
  CommandXboxController commandXboxController = new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);

  Climber climber = new Climber("src/main/deploy/configs/climber/climberConf.json");
  NoteElevator noteElevator = new NoteElevator("src/main/deploy/configs/notevator/notevatorConf.json");

  public RobotContainer() {
    configureBindings();
    configShuffleboard();
  }

  private void configureBindings() {
    // commandXboxController.y().onTrue(climber.);
    // commandXboxController.x().onTrue(climber.);

    // commandXboxController.y().onTrue(climber.)
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void onEnable() {
    indexerSubsystem.onEnable();
    shooterSubsystem.onEnable();
  }

  public void onDisable() {
    indexerSubsystem.onDisable();
    shooterSubsystem.onDisable();
  }

  public void disabledPeriodic() {
    indexerSubsystem.disabledPeriodic();
    shooterSubsystem.disabledPeriodic();
  }

  public void simulationInit() {
    indexerSubsystem.simulationInit();
    shooterSubsystem.simulationInit();
  }

  public void simulationPeriodic() {
    indexerSubsystem.simulationPeriodic();
    shooterSubsystem.simulationPeriodic();
  }

  public void configShuffleboard() {
    indexerSubsystem.configShuffleboard();
    shooterSubsystem.configShuffleboard();
  }
}
