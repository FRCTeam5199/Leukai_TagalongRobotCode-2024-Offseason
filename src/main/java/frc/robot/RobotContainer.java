// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.commands.base.ElevatorHeights;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.templateSubsystems.FlywheelTestSubsystem;
import frc.robot.templateSubsystems.LinearTestSubsystem;
import frc.robot.templateSubsystems.PivotTestSubsystem;
import frc.robot.templateSubsystems.RollerTestSubsystem;
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
    public static double shooterRPS = 60;
    // The robot's subsystems and commands are defined here...
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private double MaxAngularRate = 2.5 * Math.PI;
    private final SwerveRequest.FieldCentric fieldCentricSwerveDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
    private static boolean isIdling = false;
    private static boolean isShooting = false;
    private static boolean hasUpdatedAutoAimShot = false;
    private static Timer timer;
    private double distance;
    private double angleOffset;
    public static double driveAngleOffset;

    //    RollerTestSubsystem rollerTestSubsystem = new RollerTestSubsystem();
//    FlywheelTestSubsystem flywheelTestSubsystem = new FlywheelTestSubsystem();
//    LinearTestSubsystem linearTestSubsystem = new LinearTestSubsystem();
    PivotTestSubsystem pivotTestSubsystem = new PivotTestSubsystem();

    public RobotContainer() {
        threePieceRedExtended = autos.twoAndAHalfPieceRedExtended();
        threePieceBlueExtended = autos.twoAndAHalfPieceBlueExtended();
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

        angleOffset = UserInterface.getInstance().getShooterOffset();
        timer = new Timer();
        timer.restart();

        leftTriggerOnTrue = new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(Mode.SHOOTER, new ParallelCommandGroup(
                                        ScoreCommands.driveAutoTurn(commandXboxController.getLeftX(), commandXboxController.getLeftY(),
                                                fieldCentricSwerveDrive),
                                        new InstantCommand(() -> shooterSubsystem.setShooterSpeeds(shooterRPS)),
                                        armAutoAim,
                                        new InstantCommand(() -> timer.restart()),
                                        new InstantCommand(() -> hasUpdatedAutoAimShot = false),
                                        new InstantCommand(() -> isShooting = true)
                                )
                        ),
                        Map.entry(Mode.AMP, ScoreCommands.moveElevatorToSetpoint(ElevatorHeights.AMP)
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
                        Map.entry(Mode.SHUTTLE, ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.HIGH_SHUTTLE, 52.8935)
                                .alongWith(ScoreCommands.highShuttleAutoTurn(
                                        commandXboxController::getLeftX, commandXboxController::getLeftY,
                                        fieldCentricSwerveDrive))),
                        Map.entry(Mode.CLIMB, ClimberCommands.setClimberPowers(-0.6).alongWith(
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
        ).alongWith(new InstantCommand(() -> isIdling = false));
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
//        commandXboxController.a().onTrue(ModeCommands.switchAmpOrClimbMode(true));
//        commandXboxController.b().onTrue(ModeCommands.switchShooterOrShuttleMode(true));
//        commandXboxController.x().onTrue(ModeCommands.switchShooterOrShuttleMode(false));
//        commandXboxController.y().onTrue(ModeCommands.switchAmpOrClimbMode(false)
//                .andThen(ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.MID, 0)));
//
//        commandXboxController.leftTrigger().onTrue(leftTriggerOnTrue).onFalse(leftTriggerOnFalse);
//        commandXboxController.rightTrigger().onTrue(
//                new ConditionalCommand(
//                        ClimberCommands.setClimberPowers(0.6),
//                        IntakeCommands.intake(),
//                        () -> mode == Mode.CLIMB
//                )
//        ).onFalse(
//                new ParallelCommandGroup(
//                        IntakeCommands.stopRollers(),
//                        ClimberCommands.setClimberPowers(0)
//                )
//        );
//        commandXboxController.leftBumper().onTrue(
//                new ConditionalCommand(
//                        ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.SUB, 60)
//                                .alongWith(new InstantCommand(() -> isIdling = false)).alongWith(new InstantCommand(() -> isShooting = true)),
//                        new ConditionalCommand(
//                                ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.LOW_SHUTTlE, 70),
//                                ScoreCommands.toggleElevator(),
//                                () -> mode == Mode.SHUTTLE
//                        ),
//                        () -> mode == Mode.SHOOTER
//                )
//        ).onFalse(
//                new ConditionalCommand(
//                        new WaitCommand(0),
//                        new ParallelCommandGroup(
//                                ScoreCommands.moveShooterToStable(),
//                                ScoreCommands.elevatorStable()
//                        ),
//                        () -> mode == Mode.CLIMB
//                ).alongWith(new InstantCommand(() -> isShooting = false))
//        );
//        commandXboxController.rightBumper().onTrue(
//                new ConditionalCommand(
//                        ScoreCommands.indexerFeedCommand(shooterRPS),
//                        new ConditionalCommand(
//                                ScoreCommands.spinRollersForAmpScore(),
//                                ScoreCommands.spinRollersForTrapScore(),
//                                () -> mode == Mode.AMP
//                        ),
//                        () -> mode == Mode.SHOOTER || mode == Mode.SHUTTLE
//                )
//        ).onFalse(
//                IntakeCommands.stopRollers().alongWith(new InstantCommand(() -> hasUpdatedAutoAimShot = false))
//        );
//                        .alongWith(new InstantCommand(() -> timer.restart()))
//
//        commandXboxController.povDown().onTrue(IntakeCommands.spinRollersForOuttake()).onFalse(IntakeCommands.stopRollers());

//        commandXboxController.a().onTrue(new InstantCommand(() -> rollerTestSubsystem.setVelocity(30)))
//                .onFalse(new InstantCommand(() -> rollerTestSubsystem.setPercent(0)));
//        commandXboxController.b().onTrue(new InstantCommand(() -> rollerTestSubsystem.setPosition(10.5)));
//
//        commandXboxController.x().onTrue(new InstantCommand(() -> flywheelTestSubsystem.setVelocity(30)))
//                .onFalse(new InstantCommand(() -> flywheelTestSubsystem.setPercent(0)));
//        commandXboxController.y().onTrue(new InstantCommand(() -> linearTestSubsystem.setPosition(2)))
//                .onFalse(new InstantCommand(() -> linearTestSubsystem.setPosition(0)));
        commandXboxController.rightTrigger().onTrue(new InstantCommand(() -> pivotTestSubsystem.setPosition(0)))
                .onFalse(new InstantCommand(() -> pivotTestSubsystem.setPosition(0)));

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
//            commandSwerveDrivetrain.getPigeon2().setYaw(new Pose2d(
//                    current.getX(),
//                    current.getY(),
//                    Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180.0d : 0)).getRotation().getDegrees());

        }));
        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Autos.autonChooserRed.getSelected();
        } else {
            return Autos.autonChooserBlue.getSelected();
        }
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
        // System.out.println("Pigeon angle: " + commandSwerveDrivetrain.getPigeon2().getYaw());
        armAutoAimAngle += angleOffset;
        armAutoAimAngle = LookUpTable.findArmAngle(distance);
//        System.out.println("Auto Aim Angle: " + armAutoAimAngle);
        if (distance > 4.5) shooterRPS = 70;
        else shooterRPS = 60;

        armAutoAim.changeSetpoint(armAutoAimAngle);
//        armAutoAim.changeSetpoint(UserInterface.getInstance().getShooterPositionComponentData());

        if (Math.abs(prevArmAngle - armAutoAimAngle) > .5) {
            prevArmAngle = armAutoAimAngle;
        }

        if (isShooting && !hasUpdatedAutoAimShot
                && Math.abs(shooterSubsystem.getPivot().getPivotAbsolutePositionRot() * 360d - armAutoAimAngle) > .5 && timer.get() > .75) {
            armAutoAim.updateSetpoint(armAutoAimAngle);
            timer.restart();
            hasUpdatedAutoAimShot = true;
            if (distance > 4.5) shooterRPS = 70;
            else shooterRPS = 60;
            shooterSubsystem.setShooterSpeeds(shooterRPS);
        }

        // System.out.println("Reached shooting conditions: " + shooterSubsystem.reachedShootingCondtions(60));
        // System.out.println("Has note in indexer: " + indexerSubsystem.isNoteInIndexer());

        if (DriverStation.getAlliance().isPresent())
            driveAngleOffset = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0;
        driveAngleOffset += LookUpTable.findDriveOffsetAngle(distance);
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red && commandSwerveDrivetrain.getPose().getY() < 4.102) {
                driveAngleOffset -= 2;
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red && commandSwerveDrivetrain.getPose().getY() > 7.5) {
                driveAngleOffset += 1;
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue && commandSwerveDrivetrain.getPose().getY() < 4.102) {
                driveAngleOffset += 1;
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue && commandSwerveDrivetrain.getPose().getY() > 7.5) {
                driveAngleOffset -= 2;
            }
        }
//        System.out.println(driveAngleOffset);
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
