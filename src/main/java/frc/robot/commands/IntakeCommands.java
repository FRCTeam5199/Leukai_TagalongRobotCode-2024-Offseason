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

}

