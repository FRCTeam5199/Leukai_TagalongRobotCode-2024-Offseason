package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.LookUpTable;

public class ScoreCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final Shooter shooterSubsystem = Shooter.getInstance();

    public static Command elevatorStable() {
        return new ParallelCommandGroup(
                new ElevatorRaiseToCommand<>(elevatorSubsystem, ElevatorHeights.STABLE),
                new InstantCommand(() -> indexerSubsystem.setRollerSpeeds(0, 0, 0))
        );
    }

    public static Command moveElevatorToSetpoint(ElevatorHeights elevatorHeights) {
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, elevatorHeights);
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

    public static Command basicAutoShootCommand(double targetSpeed) {
        return new SequentialCommandGroup(
                new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.MID, true),
                flywheelSpinupCommand(targetSpeed)
        );
    }

    public static Command flywheelSpinupCommand(double targetSpeed) {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.getFlywheel(0).setFlywheelControl(targetSpeed, true);
                    shooterSubsystem.getFlywheel(1).setFlywheelControl(targetSpeed, true);
                },
                () -> {
                },
                interrupted -> {
                    shooterSubsystem.getFlywheel(0).setFlywheelPower(0);
                    shooterSubsystem.getFlywheel(1).setFlywheelPower(0);
                },
                () -> false,
                shooterSubsystem);
    }

    public static Command indexerFeedCommand() {
        return new FunctionalCommand(
                () -> {
                    indexerSubsystem.getRoller(1).setRollerPower(0.5);
                    indexerSubsystem.getRoller(2).setRollerPower(1);
                },
                () -> {
                },
                interrupted -> {
                    indexerSubsystem.getRoller(1).setRollerPower(0);
                    indexerSubsystem.getRoller(2).setRollerPower(0);
                },
                () -> false,
                indexerSubsystem);
    }

    public static Command autoAim() {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    new PivotToCommand<>(shooterSubsystem.getPivot(), LookUpTable.findValue(0).armAngle, true);
                    shooterSubsystem.getFlywheel(0).setFlywheelPower(LookUpTable.findValue(0).shooterRPM);
                    shooterSubsystem.getFlywheel(1).setFlywheelPower(LookUpTable.findValue(0).shooterRPM);
                },
                interrupted -> {
                    new PivotToCommand<>(shooterSubsystem.getPivot(), 0, true);
                    shooterSubsystem.getFlywheel(0).setFlywheelPower(0);
                    shooterSubsystem.getFlywheel(1).setFlywheelPower(0);
                },
                () -> !indexerSubsystem.isNoteInIndexer()
        );
    }
}
