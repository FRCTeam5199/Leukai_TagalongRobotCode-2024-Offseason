package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.LookUpTable;

public class ScoreCommands {
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static final AmpTrap ampTrapSubsystem = AmpTrap.getInstance();
    
    private static final TrapezoidProfile driveRotationalTrapezoidProfile = new TrapezoidProfile(new Constraints(180, 360));

    public static Command driveAutoTurn(CommandXboxController commandXboxController, FieldCentric fieldCentricSwerveDrive) {
        return new ConditionalCommand(
                turnDriveToSpeaker(commandXboxController, fieldCentricSwerveDrive, 16.58, 5.59),
                turnDriveToSpeaker(commandXboxController, fieldCentricSwerveDrive, -0.0381, 5.48),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
    }

    private static Command turnDriveToSpeaker(CommandXboxController commandXboxController, FieldCentric fieldCentricSwerveDrive, double locX, double locY) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    State currentState = new State(commandSwerveDrivetrain.getPose().getRotation().getDegrees(),
                            commandSwerveDrivetrain.getPigeon2().getRate());
                    State goalState = new State(Units.radiansToDegrees(
                            Math.atan(
                                    (locY - commandSwerveDrivetrain.getPose().getY()) /
                                            (locX - commandSwerveDrivetrain.getPose().getX()))), 0);
                    commandSwerveDrivetrain.setControl(
                            fieldCentricSwerveDrive
                                    .withVelocityX(commandXboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                                    .withVelocityY(commandXboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                                    .withRotationalRate(driveRotationalTrapezoidProfile.calculate(0.02, currentState, goalState).velocity));
                },
                interrupted -> {
                },
                () -> {
                    return false;
                },
                commandSwerveDrivetrain
        );
    }

    public static Command elevatorStable() {
        return new ParallelCommandGroup(
                new ElevatorRaiseToCommand<>(ampTrapSubsystem, ElevatorHeights.STABLE),
                new InstantCommand(() -> indexerSubsystem.setRollerSpeeds(0, 0, 0))
        );
    }

    public static Command moveElevatorToSetpoint(ElevatorHeights elevatorHeight) {
        return new ElevatorRaiseToCommand<>(ampTrapSubsystem, elevatorHeight, true);
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

    public static double getDistance(double[] robotCoords, double[] speakerCoords) {
        return Math.sqrt(Math.pow((robotCoords[1] - speakerCoords[1]), 2) + Math.pow((robotCoords[0] - speakerCoords[0]), 2));
    }

    public static Command moveShooterToAutoAim(double targetSpeed) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    double distance;
                    double[] robotCoords = new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()};
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
                () -> (shooterSubsystem.reachedShootingCondtions(targetSpeed) && !indexerSubsystem.isNoteInIndexer()),
                shooterSubsystem
        ).unless(() -> !indexerSubsystem.isNoteInIndexer());
    }

    public static Command moveShooterToAutoAimAndAutoShoot(double targetSpeed) {
        return new SequentialCommandGroup(
                new FunctionalCommand(
                        () -> {
                        },
                        () -> {
                            double distance;
                            double[] robotCoords = new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()};
                            if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                                distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
                            else
                                distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);

                            double armAngle = LookUpTable.findValue(distance);
                            shooterSubsystem.moveShooterToSetpointAndSpeed(armAngle, targetSpeed);
                            shooterSubsystem.followLastPivotProfile();

                            if (shooterSubsystem.reachedShootingCondtions(targetSpeed)) {
                                indexerSubsystem.setRollerSpeeds(0, -80, 40);
                            }
                        },
                        interrupted -> {
                            shooterSubsystem.moveShooterToSetpointAndSpeed(ShooterPivotAngles.STABLE.getRotations(), 0);
                            shooterSubsystem.getPivot().setHoldPivotPosition(true);
                            indexerSubsystem.setRollerSpeeds(0, 0, 0);
                        },
                        () -> (shooterSubsystem.reachedShootingCondtions(targetSpeed) && !indexerSubsystem.isNoteInIndexer()),
                        shooterSubsystem
                )
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
                () -> {},
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> (shooterSubsystem.reachedShootingCondtions(targetSpeed) && !indexerSubsystem.isNoteInIndexer()),
                indexerSubsystem);
    }

    public static Command trapCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                ampTrapSubsystem.setRollerPower(0.5);
                new WaitCommand(0.5);
                ampTrapSubsystem.setRollerPower(0);
            },
            interrupted -> {
                ampTrapSubsystem.setRollerPower(0);
            },
            () -> false);
    }
}
