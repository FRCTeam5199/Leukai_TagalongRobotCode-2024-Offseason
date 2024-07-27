package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;

public class ScoreCommands {
    private static final IndexerSubsystem indexerSubsystem = RobotContainer.indexerSubsystem;
    private static final NoteElevator elevatorSubsystem = RobotContainer.noteElevator;

    public static Command elevatorStable() {
        return new ParallelCommandGroup(
                new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 0),
                new InstantCommand(() -> indexerSubsystem.setRollerSpeeds(0, 0, 0))
        );
    }

    public static Command ampScore() {
        return new SequentialCommandGroup(
                new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 30),
                new FunctionalCommand(
                        () -> indexerSubsystem.setRollerSpeeds(0, 0.4, 0),
                        () -> {
                        },
                        interrupted -> elevatorStable(),
                        () -> false,
                        indexerSubsystem
                ).until(() -> !indexerSubsystem.isNoteInAmpTrap())
        ).unless(() -> !indexerSubsystem.isNoteInAmpTrap());
    }
}
