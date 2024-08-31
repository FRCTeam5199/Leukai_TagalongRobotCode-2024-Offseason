package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static final Timer timer = new Timer();

    private static Command setElevatorToStable() {
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 0);
    }

    public static Command setShooterPivotToSetpoint(ShooterPivotAngles angle) {
        return new PivotToCommand<>(shooterSubsystem, angle.getRotations(), true);
    }

    private static Command spinRollersForIntake() {
        return new FunctionalCommand(
                () -> {
                    if (indexerSubsystem.getAmpMode()) {
                        indexerSubsystem.setRollerSpeeds(120, 60, 0);
                    } else {
                        indexerSubsystem.setRollerSpeeds(120, -60, 10);
                    }
                },
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerPowers(0, 0, 0),
                () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap()),
                indexerSubsystem
        );
    }

    private static Command resettleNoteBackwards() {
        return new FunctionalCommand(
                () -> {
                    timer.restart();
                    indexerSubsystem.setRollerPowers(-5, 10, -5);
                },
                () -> {
                    System.out.println("Timer: " + timer.get());
                },
                interrupted -> indexerSubsystem.setRollerPowers(0, 0, 0),
                () -> timer.get() > .2,
                indexerSubsystem
        );
    }

    private static Command resettleNoteForwards() {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(5, -10, 5),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerPowers(0, 0, 0),
                indexerSubsystem::isNoteInIndexer,
                indexerSubsystem
        );
    }

    public static Command stopRollers() {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerPowers(0, 0, 0),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerPowers(0, 0, 0),
                indexerSubsystem::isNoteInIndexer,
                indexerSubsystem
        );
    }

    public static Command intake() {
        return new SequentialCommandGroup(
                setElevatorToStable(),
                spinRollersForIntake()
        ).unless(
                () -> (indexerSubsystem.isNoteInAmpTrap())
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
