// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem("configs/indexer/indexerConf.json");
  Shooter shooterSubsystem = new Shooter("configs/shooter/shooterConf");
  public RobotContainer() {
    configureBindings();
    configShuffleboard();
  }

  private void configureBindings() {}

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
