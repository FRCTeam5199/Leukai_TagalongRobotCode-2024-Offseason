package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.Mode;

public class IntakeCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static final Timer timer = new Timer();

    public static Command setElevatorToStable() {
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, () -> 0);
    }

    public static Command setShooterPivotToSetpoint(ShooterPivotAngles angle) {
        return new PivotToCommand<>(shooterSubsystem, angle.getRotations(), true);
    }

    private static Command spinRollersForIntake() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.getMode() == Mode.AMP || RobotContainer.getMode() == Mode.CLIMB) {
                        indexerSubsystem.setRollerSpeeds(100, 60, 0);
                    } else {
                        indexerSubsystem.setRollerSpeeds(100, -60, 10);
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
    
    public static Command spinRollersForOuttake() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.getMode() == Mode.AMP || RobotContainer.getMode() == Mode.CLIMB) {
                        indexerSubsystem.setRollerSpeeds(-100, -60, 0);
                    } else {
                        indexerSubsystem.setRollerSpeeds(-100, 60, -10);
                    }
                },
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerPowers(0, 0, 0),
                () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap()),
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

}
