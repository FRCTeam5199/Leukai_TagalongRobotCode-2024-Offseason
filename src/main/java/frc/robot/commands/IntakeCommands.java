package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
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

    private static Command setElevatorToStable() {
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 0);
    }

    public static Command setShooterPivotToSetpoint(ShooterPivotAngles angle) {
        return new PivotToCommand<>(shooterSubsystem, angle, true);
    }

    private static Command spinRollersForIntake() {
        return new FunctionalCommand(
                () -> {
                    if (indexerSubsystem.getAmpMode()) {
                        indexerSubsystem.setRollerSpeeds(1000, 2000, 0);
                    } else {
                        indexerSubsystem.setRollerSpeeds(1000, -2000, 500);
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
//                setShooterPivotToStable(),
                setElevatorToStable(),
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
                        setElevatorToStable(),
                        new FunctionalCommand(
                                () -> indexerSubsystem.setRollerSpeeds(-2500, 1500, -1000),
                                () -> {
                                    if (!indexerSubsystem.isNoteInIntake()) {
                                        indexerSubsystem.setRollerSpeeds(1500, 2500, 0);
                                    }
                                },
                                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                                indexerSubsystem::isNoteInAmpTrap,
                                indexerSubsystem
                        )
                ).unless(() -> !indexerSubsystem.isNoteInIndexer())
        );
    }

    public static Command switchShooterMode() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> indexerSubsystem.setAmpMode(false)),
                new SequentialCommandGroup(
                        setElevatorToStable(),
                        new FunctionalCommand(
                                () -> indexerSubsystem.setRollerSpeeds(-2000, -2000, 0),
                                () -> {
                                    if (indexerSubsystem.isNoteInIntake()) {
                                        indexerSubsystem.setRollerSpeeds(1000, -3000, 500);
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
