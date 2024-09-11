package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.AmpTrapSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem.LEDMode;

public class IntakeCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final AmpTrapSubsystem ampTrapSubsystem = AmpTrapSubsystem.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static final Timer timer = new Timer();

    private static Command setElevatorToStable() {
        return new ElevatorRaiseToCommand<>(ampTrapSubsystem, () -> 0);
    }

    public static Command setShooterPivotToSetpoint(ShooterPivotAngles angle) {
        return new PivotToCommand<>(shooterSubsystem, angle.getRotations(), true);
    }

    private static Command spinRollersForIntake() {
        return new FunctionalCommand(
                () -> {
                    if (indexerSubsystem.getAmpMode()) {
                        indexerSubsystem.setRollerSpeeds(100, 60, 0);
                        LEDSubsystem.getInstance().setMode(LEDMode.AMPTRAP);
                    } else {
                        indexerSubsystem.setRollerSpeeds(100, -60, 10);
                        LEDSubsystem.getInstance().setMode(LEDMode.SHOOTING);
                    }
                },
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap()),
                indexerSubsystem
        );
    }

    private static Command resettleNoteBackwards() {
        return new FunctionalCommand(
                () -> {
                    timer.restart();
                    indexerSubsystem.setRollerSpeeds(-5, 10, -5);
                },
                () -> {
                    System.out.println("Timer: " + timer.get());
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> timer.get() > .5,
                indexerSubsystem
        );
    }

    private static Command resettleNoteForwards() {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(5, -10, 5),
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
                        resettleNoteForwards(),
                        resettleNoteBackwards(),
                        resettleNoteForwards(),
                        resettleNoteBackwards(),
                        resettleNoteForwards(),
                        resettleNoteBackwards(),
                        resettleNoteForwards(),
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
                                        LEDSubsystem.getInstance().setMode(LEDMode.AMPTRAP);
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
                                        LEDSubsystem.getInstance().setMode(LEDMode.SHOOTING);
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
