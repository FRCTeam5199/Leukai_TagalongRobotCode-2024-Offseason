package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonCommands extends Command {
  private static AutonCommands autonCommands;

  private static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private static ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

  private AutonCommands() {}

  public AutonCommands getInstance() {
    if (autonCommands == null) { autonCommands = new AutonCommands(); }
    return autonCommands;
  }
}