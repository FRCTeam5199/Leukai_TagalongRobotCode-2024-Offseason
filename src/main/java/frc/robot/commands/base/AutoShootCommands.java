package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Shooter;

public class AutoShootCommands extends Command {
  public Command BasicAutoShootCommand(Shooter shooter, IndexerSubsystem indexerSubsystem) {
    return new SequentialCommandGroup(
        new PivotToCommand(shooter.getPivot(), 15.0, true),
        new WaitUntilCommand(shooter::reachedShootingConditions),
        IndexerFeedCommand(indexerSubsystem).until(shooter::shotNote)
      ).deadlineWith(FlywheelSpinupCommand(shooter));
  }

 public Command FlywheelSpinupCommand(Shooter shooter) {
  return new FunctionalCommand(() -> {
      shooter.getFlywheel(0).setFlywheelControl(3000, true);
      shooter.getFlywheel(1).setFlywheelControl(4000, true);
    }, () -> {}, interrupted -> {
      shooter.getFlywheel(0).setFlywheelPower(0);
      shooter.getFlywheel(0).setFlywheelPower(0);
    }, () -> false, shooter);
  }

  public Command IndexerFeedCommand(IndexerSubsystem indexerSubsystem) {
    return new FunctionalCommand(() -> indexerSubsystem.getRoller().setRollerPower(0.5),
      () -> {}, interrupted -> {
        indexerSubsystem.getRoller().setRollerPower(0);
      }, () -> false, indexerSubsystem);
    }
}