package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.commands.base.RollerRotateXCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;

public class IntakeCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final Shooter shooterSubsystem = Shooter.getInstance();

    private static Command moveElevatorDown() {
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 0);
    }

    private static Command moveShooterStable() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.STABLE, true);
    }

    private static Command spinRollersForIntake() {
        return new FunctionalCommand(
                () -> {
                    if (indexerSubsystem.getAmpMode()) {
                        indexerSubsystem.setRollerSpeeds(0.8, 1, 0);
                    } else {
                        indexerSubsystem.setRollerSpeeds(0.8, -1, -0.05);
                    }
                },
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap()),
                indexerSubsystem
        );
    }

    private static Command resettleNoteAfterIntaking() {
        return new ConditionalCommand(
                new RollerRotateXCommand<>(indexerSubsystem.getRoller(1), -0.05),
                new RollerRotateXCommand<>(indexerSubsystem.getRoller(2), -0.05),
                indexerSubsystem::getAmpMode

        );
    }

    public static Command intake() {
        return new SequentialCommandGroup(
                moveElevatorDown(),
                spinRollersForIntake().until(
                        () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap())
                ),
                resettleNoteAfterIntaking()
        ).unless(
                () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap())
        );
    }

    public static Command switchAmpMode() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> indexerSubsystem.setAmpMode(true)),
                new SequentialCommandGroup(
                        moveElevatorDown(),
                        moveShooterStable(),
                        new FunctionalCommand(
                                () -> indexerSubsystem.setRollerSpeeds(-0.35, 0.25, -0.5),
                                () -> {
                                },
                                interrupted -> indexerSubsystem.setRollerSpeeds(0.4, 1, 0),
                                () -> !indexerSubsystem.isNoteInIntake(),
                                indexerSubsystem
                        )
                ).unless(() -> !indexerSubsystem.isNoteInIndexer())
        );
    }

    public static Command switchShooterMode() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> indexerSubsystem.setAmpMode(false)),
                new SequentialCommandGroup(
                        moveElevatorDown(),
                        moveShooterStable(),
                        new FunctionalCommand(
                                () -> indexerSubsystem.setRollerSpeeds(-0.35, -0.5, 0.5),
                                () -> {
                                },
                                interrupted -> indexerSubsystem.setRollerSpeeds(0.4, 1, 0),
                                indexerSubsystem::isNoteInIntake,
                                indexerSubsystem
                        )
                ).unless(() -> !indexerSubsystem.isNoteInAmpTrap())
        );
    }
}
