package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.commands.base.ClimberHeights;
import frc.robot.commands.base.ElevatorHeights;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ShooterSubsystem;

public class TrapCommands {
    public static NoteElevator noteElevator;
    public static ShooterSubsystem shooter;
    public static Climber climber;

    public static Command trapPrep() {
        return new SequentialCommandGroup(
                new ElevatorRaiseToCommand<>(noteElevator, ElevatorHeights.TRAP),
                new ClimberCommands().climberAscendCommand(ClimberHeights.UP_LEFT, ClimberHeights.UP_RIGHT)
        );
    }

    public Command trapClimb(IndexerSubsystem indexer, ShooterSubsystem shooter, Climber climber) {
        return new SequentialCommandGroup(
                new ClimberCommands().climberDescendCommand(ClimberHeights.UP_LEFT, ClimberHeights.UP_RIGHT)
        );
    }


}
