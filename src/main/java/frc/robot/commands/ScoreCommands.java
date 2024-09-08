package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.UserInterface;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem.LEDMode;
import frc.robot.utility.LookUpTable;

public class ScoreCommands {
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final AmpTrap elevatorSubsystem = AmpTrap.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static PIDController driveRotationalPIDController;

    public static Command driveAutoTurn(CommandXboxController commandXboxController, FieldCentric fieldCentricSwerveDrive) {
        return new ConditionalCommand(
                driveAutoTurn(commandXboxController, fieldCentricSwerveDrive, 16.58, 5.59, 180),
                driveAutoTurn(commandXboxController, fieldCentricSwerveDrive, -0.0381, 5.48, 0),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
    }

    private static Command driveAutoTurn(CommandXboxController commandXboxController, FieldCentric fieldCentricSwerveDrive, double targetX, double targetY, double rotationalOffset) {
        return new FunctionalCommand(
                () -> driveRotationalPIDController = new PIDController(0.2, 0, 0),
                () -> {
                    commandSwerveDrivetrain.setControl(
                            fieldCentricSwerveDrive
                                    .withVelocityX(-commandXboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                                    .withVelocityY(-commandXboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                                    .withRotationalRate(driveRotationalPIDController.calculate(
                                            commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(rotationalOffset)).getDegrees(),
                                            Units.radiansToDegrees(Math.atan(
                                                    (targetY - commandSwerveDrivetrain.getPose().getY()) / (targetX - commandSwerveDrivetrain.getPose().getX()))))));
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
                new ElevatorRaiseToCommand<>(elevatorSubsystem, ElevatorHeights.STABLE),
                new InstantCommand(() -> indexerSubsystem.setRollerSpeeds(0, 0, 0))
        );
    }

    public static Command moveElevatorToSetpoint(ElevatorHeights elevatorHeight) {
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, elevatorHeight, true);
    }

    public static Command moveShooterToStable() {
        return new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true).beforeStarting(() -> {
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

    public static Command setShooterSpeeds(double rps) {
        return new FunctionalCommand(
                () -> {
                    if (rps == 0) {
                        shooterSubsystem.setFlywheelPowers(0);
                    } else {
                        shooterSubsystem.setShooterSpeeds(rps);
                    }
                },
                () -> {

                },
                interrupted -> shooterSubsystem.setFlywheelPowers(0),
                () -> false,
                shooterSubsystem
        );
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
                                LEDSubsystem.getInstance().setMode(LEDMode.REACHEDSHOOTINGSPEED);
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

//    public static Command shoot(double targetSpeed) {
//        return new ConditionalCommand(
//                ampScore(),
//                moveShooterToAutoAim(targetSpeed),
//                indexerSubsystem::getAmpMode
//        );
//    }

    public static Command moveShooterToSetpointAndSpeed(ShooterPivotAngles shooterPivotAngle, double targetSpeed) {
        return new PivotToCommand<>(shooterSubsystem, shooterPivotAngle.getRotations(), true).beforeStarting(() -> {
            if (targetSpeed == 0) {
                shooterSubsystem.getFlywheel(0).setFlywheelPower(0);
                shooterSubsystem.getFlywheel(1).setFlywheelPower(0);
            } else {
                shooterSubsystem.getFlywheel(0).setFlywheelControl(.57 * targetSpeed, true);
                shooterSubsystem.getFlywheel(1).setFlywheelControl(targetSpeed, true);
            }
        });
    }

    public static Command generateLookUpTable(double targetSpeed) {
        return new PivotToCommand<>(shooterSubsystem, Rotation2d.fromDegrees(UserInterface.getInstance().getShooterPositionComponentData()).getRotations(), true).beforeStarting(() -> {
            shooterSubsystem.getFlywheel(0).setFlywheelControl(.57 * targetSpeed, true);
            shooterSubsystem.getFlywheel(1).setFlywheelControl(targetSpeed, true);
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
}
