package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.Mode;

public class ModeCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public static Command switchAmpOrClimbMode(boolean isAmpMode) {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new InstantCommand(() -> RobotContainer.setMode(Mode.AMP)),
                        new InstantCommand(() -> RobotContainer.setMode(Mode.CLIMB)),
                        () -> isAmpMode
                ),
                new SequentialCommandGroup(
                        IntakeCommands.setElevatorToStable(),
                        new FunctionalCommand(
                                () -> indexerSubsystem.setRollerSpeeds(-40, 25, -20),
                                () -> {
                                    if (!indexerSubsystem.isNoteInIntake()) {
                                        indexerSubsystem.setRollerSpeeds(25, 40, 0);
                                    }
                                },
                                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                                indexerSubsystem::isNoteInAmpTrap,
                                indexerSubsystem
                        )
                ).unless(() -> !indexerSubsystem.isNoteInIndexer())
        );
    }


    public static Command switchShooterOrShuttleMode(boolean isShooterMode) {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new InstantCommand(() -> RobotContainer.setMode(Mode.SHOOTER)),
                        new InstantCommand(() -> RobotContainer.setMode(Mode.SHUTTLE)),
                        () -> isShooterMode
                ), new SequentialCommandGroup(
                IntakeCommands.setElevatorToStable(),
                new FunctionalCommand(
                        () -> indexerSubsystem.setRollerSpeeds(-10, -20, 0),
                        () -> {
                            if (indexerSubsystem.isNoteInIntake()) {
                                indexerSubsystem.setRollerSpeeds(25, -50, 10);
                            }
                        },
                        interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                        indexerSubsystem::isNoteInIndexer,
                        indexerSubsystem
                )
        ).unless(() -> !indexerSubsystem.isNoteInAmpTrap())
        );
    }

}
