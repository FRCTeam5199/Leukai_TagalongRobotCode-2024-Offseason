package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.UserInterface;
import frc.robot.commands.base.ElevatorHeights;
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

import java.util.function.Supplier;

public class ScoreCommands {
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static final AmpTrap elevatorSubsystem = AmpTrap.getInstance();
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static PIDController driveRotationalPIDController;
    public static boolean isElevatorUp = false;
    public static double driveBaseTarget = 0;

    public static Command driveAutoTurn(double driveX, double driveY, FieldCentric fieldCentricSwerveDrive) {
        return new ConditionalCommand(
                driveAutoTurn(() -> 0.0, () -> 0.0, fieldCentricSwerveDrive, 16.58, 5.59),
                driveAutoTurn(() -> 0.0, () -> 0.0, fieldCentricSwerveDrive, -.0381, 5.48),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        );
    }

    // private static double getAutoAimOffset() {
    //     // offset.addOption("normal", true);
    //     // offset.addOption("weird", false);
    //     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //         if (commandSwerveDrivetrain.getPose().getY() < 4.102) return 183;x
    //         else return 178;
    //     } else {
    //         if (commandSwerveDrivetrain.getPose().getY() < 4.102) return 0;
    //         else return 0;  
    //     }
    // }

    public static Command toggleElevator() {
        return new ConditionalCommand(
                elevatorStable(),
                new InstantCommand(() -> isElevatorUp = true).andThen(moveElevatorToSetpoint(ElevatorHeights.TRAP)),
                () -> isElevatorUp
        );
    }

    public static Command isElevatorUp(boolean up) {
        return new InstantCommand(() -> isElevatorUp = up);
    }

    private static Command driveAutoTurn(Supplier<Double> driveX, Supplier<Double> driveY, FieldCentric fieldCentricSwerveDrive, double targetX, double targetY) {
        return new FunctionalCommand(
                () -> driveRotationalPIDController = new PIDController(0.175, 0, 0),
                () -> {
                    commandSwerveDrivetrain.setControl(
                            fieldCentricSwerveDrive
                                    .withVelocityX(-driveY.get() * TunerConstants.kSpeedAt12VoltsMps)
                                    .withVelocityY(-driveX.get() * TunerConstants.kSpeedAt12VoltsMps)
                                    .withRotationalRate(driveRotationalPIDController.calculate(
                                            commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(RobotContainer.driveAngleOffset)).getDegrees(),
                                            Units.radiansToDegrees(Math.atan(
                                                    (targetY - commandSwerveDrivetrain.getPose().getY()) / (targetX - commandSwerveDrivetrain.getPose().getX()))))));
                    // System.out.println("Current angle: " + commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(rotationalOffset)).getDegrees());
                    // System.out.println("Goal angle: " + Units.radiansToDegrees(Math.atan(
                    // (targetY - commandSwerveDrivetrain.getPose().getY()) / (targetX - commandSwerveDrivetrain.getPose().getX()))));
//                    System.out.println("drive aiming");
                },
                interrupted -> {
                },
                () -> false,
                commandSwerveDrivetrain
        );
    }

    public static Command autoShuttleAim(FieldCentric fieldCentricSwerveDrive, double targetX, double targetY, double rotationalOffset) {
        return commandSwerveDrivetrain.applyRequest(() -> fieldCentricSwerveDrive
                .withVelocityX(-RobotContainer.commandXboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                .withVelocityY(-RobotContainer.commandXboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                .withRotationalRate(driveRotationalPIDController.calculate(
                        commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(rotationalOffset)).getDegrees(),
                        Units.radiansToDegrees(Math.atan(
                                (targetY - commandSwerveDrivetrain.getPose().getY()) / (targetX - commandSwerveDrivetrain.getPose().getX()))))));
    }

    private static Command autonAutoTurn(double driveX, double driveY, FieldCentric fieldCentricSwerveDrive, double targetX, double targetY) {
        return new FunctionalCommand(
                () -> driveRotationalPIDController = new PIDController(.4, 0, 0),
                () -> {
                    commandSwerveDrivetrain.setControl(
                            fieldCentricSwerveDrive
                                    .withVelocityX(-driveY * TunerConstants.kSpeedAt12VoltsMps)
                                    .withVelocityY(-driveX * TunerConstants.kSpeedAt12VoltsMps)
                                    .withRotationalRate(driveRotationalPIDController.calculate(
                                            commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(RobotContainer.driveAngleOffset)).getDegrees(),
                                            Units.radiansToDegrees(Math.atan(
                                                    (targetY - commandSwerveDrivetrain.getPose().getY()) / (targetX - commandSwerveDrivetrain.getPose().getX()))))));
                    System.out.println("drive auto aiming");
                },
                interrupted -> {
                },
                () -> commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(RobotContainer.driveAngleOffset + .85)).getDegrees() >= Units.radiansToDegrees(Math.atan(
                        (targetY - commandSwerveDrivetrain.getPose().getY()) / (targetX - commandSwerveDrivetrain.getPose().getX()))) && commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(RobotContainer.driveAngleOffset - .85)).getDegrees() <= Units.radiansToDegrees(Math.atan(
                        (targetY - commandSwerveDrivetrain.getPose().getY()) / (targetX - commandSwerveDrivetrain.getPose().getX()))),
                commandSwerveDrivetrain
        );
    }

    public static Command autonAutoTurn(FieldCentric fieldCentric) {
        return new ConditionalCommand(
                autonAutoTurn(0, 0, fieldCentric, 16.58, 5.59),
                autonAutoTurn(0, 0, fieldCentric, -.0381, 5.48),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        );
    }

    public static Command highShuttleAutoTurn(Supplier<Double> driveX, Supplier<Double> driveY, FieldCentric fieldCentricSwerveDrive) {
        return new ConditionalCommand(
                driveAutoTurn(driveX, driveY, fieldCentricSwerveDrive, 16.58, 13d),
                driveAutoTurn(driveX, driveY, fieldCentricSwerveDrive, -15d, 4d),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
    }


    public static Command elevatorStable() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> isElevatorUp = false),
                new ElevatorRaiseToCommand<>(elevatorSubsystem, ElevatorHeights.STABLE),
                new InstantCommand(() -> indexerSubsystem.setRollerSpeeds(0, 0, 0))
        );
    }

    public static double getShotAngle() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return Units.radiansToDegrees(Math.atan((5.59 - commandSwerveDrivetrain.getPose().getY()) / (16.58 - commandSwerveDrivetrain.getPose().getX())));
        } else
            return Units.radiansToDegrees(Math.atan((5.59 - commandSwerveDrivetrain.getPose().getY()) / (16.58 - commandSwerveDrivetrain.getPose().getX())));

    }

    public static Command subWooferShot() {
        return moveShooterToSetpointAndSpeed(ShooterPivotAngles.MAX, 45).andThen(indexerFeedCommandAutoStop(45));
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

    public static Command spinRollersForTrapScore() {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(0, 100, 0),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerPowers(0, 0, 0),
                () -> false,
                indexerSubsystem
        );
    }

    public static Command spinRollersForAmpScore() {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(0, 50, 0),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerPowers(0, 0, 0),
                () -> false,
                indexerSubsystem
        );
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
                interrupted -> {
                },
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

                            double armAngle = LookUpTable.findArmAngle(distance);
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

    public static Command indexerFeedCommandAutoStop(double targetSpeed) {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(50, -75, 50),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> (shooterSubsystem.reachedShootingCondtions(targetSpeed) && !indexerSubsystem.isNoteInIndexer()),
                indexerSubsystem);
    }

    public static Command indexerFeedCommandInstant() {
        return new InstantCommand(() -> indexerSubsystem.setRollerPowers(50, -75, 50));
    }

    public static Command indexerFeedCommand(double targetSpeed) {
        return new FunctionalCommand(
                () -> indexerSubsystem.setRollerSpeeds(50, -75, 50),
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> false,
                indexerSubsystem);
    }
}
