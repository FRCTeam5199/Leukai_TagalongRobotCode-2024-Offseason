package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonCommands extends Command {
  private static AutonCommands autonCommands;

  private static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
  private static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private static ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

  private AutonCommands() {}

  public AutonCommands getInstance() {
    if (autonCommands == null) { autonCommands = new AutonCommands(); }
    return autonCommands;
  }

  public Command autoShootCommand(double targetSpeed) {
    return new FunctionalCommand(
      () -> {}, 
      () -> ScoreCommands.moveShooterToAutoAim(4000).until(() -> shooterSubsystem.reachedShootingConditions(4000)).andThen(ScoreCommands.indexerFeedCommand()),
      interrupted -> {}, 
      () -> (shooterSubsystem.shotNote(4000) && !indexerSubsystem.isNoteInIndexer()) || !indexerSubsystem.isNoteInIndexer(),
      shooterSubsystem, indexerSubsystem);
  }
}