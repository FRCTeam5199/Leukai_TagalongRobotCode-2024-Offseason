package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final AmpTrap elevatorSubsystem = AmpTrap.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

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
                        indexerSubsystem.setRollerSpeeds(100, 60, 0);
                    } else {
                        indexerSubsystem.setRollerSpeeds(100, -60, 10);
                    }
                },
                () -> {
                },
                interrupted -> {
                    // System.out.println(interrupted);
                    indexerSubsystem.setRollerSpeeds(0, 0, 0);
                },
                () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap()),
                indexerSubsystem
        );
    }

    private static Command resettleNoteBackwards() {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(0, 10, -5),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> !indexerSubsystem.isNoteInIndexer(),
                indexerSubsystem
        );
    }

    private static Command resettleNoteForwards() {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(0, -10, 5),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                indexerSubsystem::isNoteInIndexer,
                indexerSubsystem
        );
    }

    public static Command intake() {
        return new SequentialCommandGroup(
                setElevatorToStable(),
                spinRollersForIntake().until(
                        () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap())
                ),
                new SequentialCommandGroup(
                        resettleNoteBackwards(),
                        resettleNoteForwards()
                ).unless(indexerSubsystem::isNoteInAmpTrap)
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

    public static Command switchShooterMode() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> indexerSubsystem.setAmpMode(false)),
                new SequentialCommandGroup(
                        setElevatorToStable(),
                        new FunctionalCommand(
                                () -> indexerSubsystem.setRollerSpeeds(-25, -50, 0),
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
