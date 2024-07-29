package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.LookUpTable;

import java.sql.Driver;

public class ScoreCommands {
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final Shooter shooterSubsystem = Shooter.getInstance();
    private static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    public static Command elevatorStable() {
        return new ParallelCommandGroup(
                new ElevatorRaiseToCommand<>(elevatorSubsystem, ElevatorHeights.STABLE),
                new InstantCommand(() -> indexerSubsystem.setRollerSpeeds(0, 0, 0))
        );
    }

    public static Command moveElevatorToSetpoint(ElevatorHeights elevatorHeight) {
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, elevatorHeight, true);
    }

    public static Command moveShooterToStable() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.STABLE, true).beforeStarting(() -> {
            shooterSubsystem.getFlywheel(0).setFlywheelPower(0);
            shooterSubsystem.getFlywheel(1).setFlywheelPower(0);
            indexerSubsystem.getRoller(1).setRollerPower(0);
            indexerSubsystem.getRoller(2).setRollerPower(0);
        });
    }

    public static Command ampScore() {
        return new SequentialCommandGroup(
                moveElevatorToSetpoint(ElevatorHeights.AMP),
                new FunctionalCommand(
                        () -> indexerSubsystem.setRollerSpeeds(0, 30, 0),
                        () -> {
                        },
                        interrupted -> elevatorStable(),
                        () -> false,
                        indexerSubsystem
                ).until(() -> !indexerSubsystem.isNoteInAmpTrap())
        ).unless(() -> !indexerSubsystem.isNoteInAmpTrap());
    }

    private static double getDistance(double[] robotCoords, double[] speakerCoords) {
        return Math.sqrt(Math.pow((robotCoords[1] - speakerCoords[1]), 2) + Math.pow((robotCoords[0] - speakerCoords[0]), 2));
    }

    public static Command moveShooterToAutoAim(double targetSpeed) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    if (DriverStation.getAlliance().isPresent()) {
                        double distance;
                        double[] robotCoords = new double[]{drivetrain.getPose().getX(), drivetrain.getPose().getY()};
                        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                            distance = getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
                        else
                            distance = getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);

                        double armAngle = LookUpTable.findValue(distance);
                        shooterSubsystem.moveShooterToSetpointAndSpeed(armAngle, targetSpeed);
                        shooterSubsystem.followLastPivotProfile();
                    }
                },
                interrupted -> shooterSubsystem.moveShooterToSetpointAndSpeed(ShooterPivotAngles.STABLE.getDegrees(), 0),
                () -> false,
                shooterSubsystem
        ).unless(() -> !indexerSubsystem.isNoteInIndexer());
    }

    public static Command shoot(double targetSpeed) {
        return new ConditionalCommand(
                ampScore(),
                moveShooterToAutoAim(targetSpeed),
                indexerSubsystem::getAmpMode
        );
    }

    public static Command moveShooterToSetpointAndSpeed(ShooterPivotAngles shooterPivotAngle, double targetSpeed) {
        return new PivotToCommand<>(shooterSubsystem, shooterPivotAngle, true).beforeStarting(() -> {
            shooterSubsystem.getFlywheel(0).setFlywheelControl(targetSpeed, true);
            shooterSubsystem.getFlywheel(1).setFlywheelControl(.5 * targetSpeed, true);
        });
    }

    public static Command indexerFeedCommand() {
        return new FunctionalCommand(
                () -> {
                    indexerSubsystem.getRoller(1).setRollerPower(1);
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
}
