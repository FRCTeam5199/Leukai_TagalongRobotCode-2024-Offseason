// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.commands.base.ElevatorHeights;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.LookUpTable;
import frc.robot.utility.Mode;

import java.util.Map;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(
            Ports.DRIVER_XBOX_USB_PORT);
    public final static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain; // My drivetrain
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final Climber climberSubsystem = Climber.getInstance();
    public static final AmpTrap ampTrap = AmpTrap.getInstance();
    public final static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();
    public static final Autos autos = new Autos(commandSwerveDrivetrain);
    // driving in open loop
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static double armAutoAimAngle;
    private static Mode mode = Mode.SHOOTER;
    public double prevArmAngle = 0;
    public PivotToCommand armAutoAim = new PivotToCommand(
            shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true
    );
    private final Command threePieceRedExtended;
    private final Command threePieceBlueExtended;
    private final Command fourPieceRed;
    private final Command fourPieceBlue;
    private final Command sixPieceRed;
    private final Command leftTriggerOnTrue;
    private final Command leftTriggerOnFalse;
    private double shooterRPS = 60;
    // The robot's subsystems and commands are defined here...
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric fieldCentricSwerveDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
    private static boolean isIdling = false;
    private static boolean isShooting = false;

    double distance;

    public RobotContainer() {
        threePieceRedExtended = autos.threePieceRedExtended();
        threePieceBlueExtended = autos.threePieceBlueExtended();
        fourPieceRed = autos.fourPieceRed();
        fourPieceBlue = autos.fourPieceBlue();
        sixPieceRed = autos.sixPieceRed();

        Shuffleboard.getTab("Shooter Tuning").addNumber("Distance", () -> distance);
        Shuffleboard.getTab("Shooter Tuning").addNumber("Target Angle", () -> armAutoAimAngle);
        Shuffleboard.getTab("Shooter Tuning").addNumber("Current Angle", () -> shooterSubsystem.getPivot().getPivotAbsolutePositionRot() * 360d);
        Shuffleboard.getTab("Shooter Tuning").addNumber("Shooter 1 Speed", () -> shooterSubsystem.getFlywheel(0).getFlywheelVelocity());
        Shuffleboard.getTab("Shooter Tuning").addNumber("Shooter 2 Speed", () -> shooterSubsystem.getFlywheel(1).getFlywheelVelocity());

        Shuffleboard.getTab("Drive Info").addBoolean("Climb Mode", () -> mode == Mode.CLIMB);
        Shuffleboard.getTab("Drive Info").addBoolean("Shoot Mode", () -> mode == Mode.SHOOTER);
        Shuffleboard.getTab("Drive Info").addBoolean("Shuttle Mode", () -> mode == Mode.SHUTTLE);


        leftTriggerOnTrue = new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(Mode.SHOOTER, new ParallelCommandGroup(
                                        ScoreCommands.driveAutoTurn(commandXboxController.getLeftX(), commandXboxController.getLeftY(),
                                                fieldCentricSwerveDrive),
                                        new InstantCommand(() -> shooterSubsystem.setShooterSpeeds(shooterRPS)),
                                        armAutoAim
                                )
                        ),
                        Map.entry(Mode.AMP, ScoreCommands.moveElevatorToSetpoint(ElevatorHeights.AMP)
                                .alongWith(ScoreCommands.isElevatorUp(true)).alongWith(
                                        commandSwerveDrivetrain.applyRequest(
                                                () -> {
                                                    return
                                                            // Drive forward with negative Y (forward)
                                                            fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                                                    // Drive left with negative X (left)
                                                                    .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                                                    // Drive counterclockwise with negative X (left)
                                                                    .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate);
                                                }
                                        )
                                )),
                        Map.entry(Mode.SHUTTLE, ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.HIGH_SHUTTLE, 50)
                                .alongWith(commandSwerveDrivetrain.applyRequest(
                                        () -> {
                                            return
                                                    // Drive forward with negative Y (forward)
                                                    fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                                            // Drive left with negative X (left)
                                                            .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                                            // Drive counterclockwise with negative X (left)
                                                            .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate);
                                        }
                                ))),
                        Map.entry(Mode.CLIMB, ClimberCommands.setClimberPowers(-0.3).alongWith(
                                commandSwerveDrivetrain.applyRequest(
                                        () -> {
                                            return
                                                    // Drive forward with negative Y (forward)
                                                    fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                                            // Drive left with negative X (left)
                                                            .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                                            // Drive counterclockwise with negative X (left)
                                                            .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate);
                                        }
                                )
                        ))
                ),
                () -> mode
        ).alongWith(new InstantCommand(() -> isIdling = false)).alongWith(new InstantCommand(() -> isShooting = true));
        leftTriggerOnFalse = new ConditionalCommand(
                ClimberCommands.setClimberPowers(0),
                new ParallelCommandGroup(
                        ScoreCommands.elevatorStable(),
                        ScoreCommands.moveShooterToStable(),
                        ClimberCommands.setClimberPowers(0)
                ),
                () -> mode == Mode.CLIMB
        ).alongWith(new InstantCommand(() -> isShooting = false));

        configureBindings();
    }

    public static Mode getMode() {
        return mode;
    }

    public static void setMode(Mode mode) {
        RobotContainer.mode = mode;
    }

    private void configureBindings() {
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(
                        () -> {
                            return
                                    // Drive forward with negative Y (forward)
                                    fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                            // Drive left with negative X (left)
                                            .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                            // Drive counterclockwise with negative X (left)
                                            .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate);
                        }
                )
        );

        //Mode Switches
        commandXboxController.a().onTrue(ModeCommands.switchAmpOrClimbMode(true));
        commandXboxController.b().onTrue(ModeCommands.switchShooterOrShuttleMode(true));
        commandXboxController.x().onTrue(ModeCommands.switchShooterOrShuttleMode(false));
        commandXboxController.y().onTrue(ModeCommands.switchAmpOrClimbMode(false)
                .andThen(ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.MID, 0)));

        commandXboxController.leftTrigger().onTrue(leftTriggerOnTrue).onFalse(leftTriggerOnFalse);
        commandXboxController.rightTrigger().onTrue(
                new ConditionalCommand(
                        ClimberCommands.setClimberPowers(0.3),
                        IntakeCommands.intake(),
                        () -> mode == Mode.CLIMB
                )
        ).onFalse(
                new ParallelCommandGroup(
                        IntakeCommands.stopRollers(),
                        ClimberCommands.setClimberPowers(0)
                )
        );
        commandXboxController.leftBumper().onTrue(
                new ConditionalCommand(
                        ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.MID, 60)
                                .alongWith(new InstantCommand(() -> isIdling = false)).alongWith(new InstantCommand(() -> isShooting = true)),
                        new ConditionalCommand(
                                ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.LOW_SHUTTlE, 70),
                                ScoreCommands.toggleElevator(),
                                () -> mode == Mode.SHUTTLE
                        ),
                        () -> mode == Mode.SHOOTER
                )
        ).onFalse(
                new ConditionalCommand(
                        new WaitCommand(0),
                        new ParallelCommandGroup(
                                ScoreCommands.moveShooterToStable(),
                                ScoreCommands.elevatorStable()
                        ),
                        () -> mode == Mode.CLIMB
                ).alongWith(new InstantCommand(() -> isShooting = false))
        );
        commandXboxController.rightBumper().onTrue(
                new ConditionalCommand(
                        ScoreCommands.indexerFeedCommand(shooterRPS),
                        new ConditionalCommand(
                                ScoreCommands.spinRollersForAmpScore(),
                                ScoreCommands.spinRollersForTrapScore(),
                                () -> mode == Mode.AMP
                        ),
                        () -> mode == Mode.SHOOTER || mode == Mode.SHUTTLE
                )
        ).onFalse(
                IntakeCommands.stopRollers()
        );

        commandXboxController.povDown().onTrue(IntakeCommands.spinRollersForOuttake()).onFalse(IntakeCommands.stopRollers());


//        commandXboxController.povUp().whileTrue(DriveCommands.goToNote());


        //Testing shooter pivot and speeds
//        commandXboxController.povDown().onTrue(ScoreCommands.setShooterSpeeds(10));
//        commandXboxController.povLeft().onTrue(ScoreCommands.setShooterSpeeds(30));
//        commandXboxController.povUp().onTrue(ScoreCommands.setShooterSpeeds(50));
//        commandXboxController.povRight().onTrue(ScoreCommands.setShooterSpeeds(90));

//        commandXboxController.povDown().onTrue(ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.STABLE, 0));
//        commandXboxController.povLeft().onTrue(ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.LOW, 0));
//        commandXboxController.povUp().onTrue(ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.MID, 0));
//        commandXboxController.povRight().onTrue(ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.MAX, 0));

        commandXboxController.button(8).onTrue(commandSwerveDrivetrain.runOnce(() -> {
            // Seed field relative pose that is alliance dependent
            var current = commandSwerveDrivetrain.getPose();
            commandSwerveDrivetrain.seedFieldRelative(
                    new Pose2d(
                            current.getX(),
                            current.getY(),
                            Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180.0d : 0)));

        }));
        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new ConditionalCommand(
                Autos.autonChooserRed.getSelected(),
                Autos.autonChooserBlue.getSelected(),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        );
    }

    public void periodic() {
        double[] robotCoords = new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()};

        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
        else
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);

        //System.out.println("Distance: " + distance);
        //System.out.println("Speed: " + shooterSubsystem.getFlywheel().getFlywheelVelocity());
//        System.out.println("Shooter sensor: " + indexerSubsystem.isNoteInIndexer());
        armAutoAimAngle = LookUpTable.findValue(distance);
//        System.out.println("Auto Aim Angle: " + armAutoAimAngle);
        if (distance > 4.5) shooterRPS = 70;
        else shooterRPS = 60;

        armAutoAim.changeSetpoint(armAutoAimAngle);
        //armAutoAim.changeSetpoint(UserInterface.getInstance().getShooterPositionComponentData());

        if (Math.abs(prevArmAngle - armAutoAimAngle) > .5) {
            Autos.aimingWhileMoving.changeSetpoint(armAutoAimAngle);

            prevArmAngle = armAutoAimAngle;
        }

        Autos.aiming.changeSetpoint(armAutoAimAngle);
    }

    public static void teleopPeriodic() {
//        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
//            if (commandSwerveDrivetrain.getPose().getX() > 10d && !isIdling && !isShooting && mode == Mode.SHOOTER) {
//                shooterSubsystem.setShooterSpeeds(30);
//                isIdling = true;
//            } else if (commandSwerveDrivetrain.getPose().getX() <= 10d && isIdling || mode != Mode.SHOOTER) {
//                shooterSubsystem.setShooterSpeeds(0);
//                isIdling = false;
//            }
//        } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
//            if (commandSwerveDrivetrain.getPose().getX() < 7d && !isIdling && !isShooting && mode == Mode.SHOOTER) {
//                shooterSubsystem.setShooterSpeeds(30);
//                isIdling = true;
//            } else if (commandSwerveDrivetrain.getPose().getX() >= 7d && isIdling || mode != Mode.SHOOTER) {
//                shooterSubsystem.setShooterSpeeds(0);
//                isIdling = false;
//            }
//        }
    }

    public void onEnable() {
        indexerSubsystem.onEnable();
        shooterSubsystem.onEnable();
        climberSubsystem.onEnable();
        ampTrap.onEnable();
    }

    public void onDisable() {
        indexerSubsystem.onDisable();
        shooterSubsystem.onDisable();
        climberSubsystem.onDisable();
        ampTrap.onEnable();

    }

    public void disabledPeriodic() {
        indexerSubsystem.disabledPeriodic();
        shooterSubsystem.disabledPeriodic();
        climberSubsystem.disabledPeriodic();
        ampTrap.disabledPeriodic();
    }

    public void simulationInit() {
        indexerSubsystem.simulationInit();
        shooterSubsystem.simulationInit();
        climberSubsystem.simulationInit();
        ampTrap.simulationInit();
    }

    public void simulationPeriodic() {
        indexerSubsystem.simulationPeriodic();
        shooterSubsystem.simulationPeriodic();
        climberSubsystem.simulationPeriodic();
        ampTrap.simulationPeriodic();
    }
}
