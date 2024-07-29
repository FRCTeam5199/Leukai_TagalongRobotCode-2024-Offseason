package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.LookUpTable;

public class ScoreCommands {
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final NoteElevator elevatorSubsystem = NoteElevator.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static TrapezoidProfile driveRotationalTrapezoidProfile;

    public static Command driveAutoTurn(CommandXboxController commandXboxController, FieldCentric fieldCentricSwerveDrive) {
        return new ConditionalCommand(
            turnDriveToSpeaker(commandXboxController, fieldCentricSwerveDrive, 0.0, 16.58, 5.59),
            turnDriveToSpeaker(commandXboxController, fieldCentricSwerveDrive, 180.0, -0.0381, 5.48),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
    }

    private static Command turnDriveToSpeaker(CommandXboxController commandXboxController, FieldCentric fieldCentricSwerveDrive, double offset, double locX, double locY) {
        return new FunctionalCommand(
            () -> {
                driveRotationalTrapezoidProfile = new TrapezoidProfile(new Constraints(180, 360));
            },
            () -> {
                State currentState = new State(commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(offset)).getDegrees(),
                        commandSwerveDrivetrain.getPigeon2().getRate());
                State goalState = new State(Units.radiansToDegrees(Math.atan(
                        (locY - commandSwerveDrivetrain.getPose().getY()) / (locX
                            - commandSwerveDrivetrain.getPose().getX()))),
                        0);
                commandSwerveDrivetrain.setControl(fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY())
                    .withVelocityY(-commandXboxController.getLeftX())
                    .withRotationalRate(driveRotationalTrapezoidProfile.calculate(0.02,
                currentState,
                goalState).velocity));
            },
            interrupted -> {},
            () -> { return false; },
            commandSwerveDrivetrain
            );
    }

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
                    double distance;
                    double[] robotCoords = new double[]{drivetrain.getPose().getX(), drivetrain.getPose().getY()};
                    if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                        distance = getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
                    else
                        distance = getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);

                    double armAngle = LookUpTable.findValue(distance);
                    shooterSubsystem.moveShooterToSetpointAndSpeed(armAngle, targetSpeed);
                    shooterSubsystem.followLastPivotProfile();
                },
                interrupted -> {
                    shooterSubsystem.moveShooterToSetpointAndSpeed(ShooterPivotAngles.STABLE.getRotations(), 0);
                    shooterSubsystem.getPivot().setHoldPivotPosition(true);
                },
                () -> (shooterSubsystem.reachedShootingConditions(targetSpeed) && !indexerSubsystem.isNoteInIndexer()),
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

    public static Command indexerFeedCommand(double targetSpeed) {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(0, -80, 40),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> (shooterSubsystem.reachedShootingConditions(targetSpeed) && !indexerSubsystem.isNoteInIndexer()),
                indexerSubsystem);
    }
}
