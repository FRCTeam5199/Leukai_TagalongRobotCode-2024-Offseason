package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonCommands {
  private static AutonCommands autonCommands;

    private static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    private AutonCommands() {
    }

    public AutonCommands getInstance() {
        if (autonCommands == null) {
            autonCommands = new AutonCommands();
        }
        return autonCommands;
    }

  public static Command autoShootCommand(double targetSpeed) {
    return new FunctionalCommand(
      () -> {}, 
      () -> ScoreCommands.moveShooterToAutoAim(60)
      .until(() -> shooterSubsystem.hasShotNote(60))
      .andThen(ScoreCommands.indexerFeedCommand(60)),
      interrupted -> {}, 
      () -> (shooterSubsystem.hasShotNote(60) && !indexerSubsystem.isNoteInIndexer()) || !indexerSubsystem.isNoteInIndexer(),
      shooterSubsystem, indexerSubsystem);
  }
}