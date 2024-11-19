// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IndexerSubsystem;
public class IntakeCommands{
  IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  public Command Intake() {
    //TODO: tune?!
    return indexerSubsystem.runOnce(
        () -> {
          indexerSubsystem.setRollerSpeeds(10,0,0);
        }).andThen(
          () -> {
            indexerSubsystem.setRollerSpeeds(0,0,0);
          }
        );
  }
  public Command stopIntake() {
      return indexerSubsystem.runOnce(() -> {
          indexerSubsystem.setRollerSpeeds(0, 0, 0);
      });
  }
  public Command Amp() {
    //TODO: tune?!
    return indexerSubsystem.runOnce(
        () -> {
          indexerSubsystem.setRollerSpeeds(0,10,0);
        }).andThen(
          () -> {
            indexerSubsystem.setRollerSpeeds(0,0,0);
          }
        );
  }
  public Command Indexer() {
    //TODO: tune?!
    return indexerSubsystem.runOnce(
        () -> {
          indexerSubsystem.setRollerSpeeds(0,0,10);
        }).andThen(
          () -> {
            indexerSubsystem.setRollerSpeeds(0,0,0);
          }
        );
  }
  public Command stopIndexer() {
      return indexerSubsystem.runOnce(
              () -> {
                  indexerSubsystem.setRollerSpeeds(0, 0, 0);
              });
  }

}
