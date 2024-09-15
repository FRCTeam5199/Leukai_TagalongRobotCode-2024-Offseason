// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.commands.base.ElevatorHeights;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.LookUpTable;
import frc.robot.utility.Mode;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(
            Ports.DRIVER_XBOX_USB_PORT);
    public final static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain; // My drivetrain
    // NoteElevator noteElevator = new
    // NoteElevator("configs/notevator/notevatorConf.json");
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final Climber climberSubsystem = Climber.getInstance();
    public static final NoteElevator noteElevator = NoteElevator.getInstance();
    public final static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();
    public static final Autos autos = new Autos(commandSwerveDrivetrain);
    // driving in open loop
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static Mode mode = Mode.SHOOTER;
    public double prevArmAngle = 0;
    public double armAutoAimAngle;
    public PivotToCommand armAutoAim = new PivotToCommand(
            shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true
    );
    private double shooterRPS = 60;
    // The robot's subsystems and commands are defined here...
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric fieldCentricSwerveDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    public RobotContainer() {
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

        commandXboxController.leftTrigger().onTrue(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                ScoreCommands.driveAutoTurn(commandXboxController.getLeftX(), commandXboxController.getLeftY(),
                                        fieldCentricSwerveDrive),
                                new InstantCommand(() -> shooterSubsystem.setShooterSpeeds(shooterRPS)),
                                armAutoAim
                        ),
                        new ConditionalCommand(
                                ScoreCommands.moveElevatorToSetpoint(ElevatorHeights.AMP)
                                        .alongWith(ScoreCommands.isElevatorUp(true)),
                                new ConditionalCommand(
                                        ClimberCommands.setClimberPowers(0.3),
                                        ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.HIGH_SHUTTLE, 60),
                                        () -> mode == Mode.CLIMB
                                ),
                                () -> mode == Mode.AMP
                        ),
                        () -> mode == Mode.SHOOTER
                )
        ).onFalse(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                ScoreCommands.elevatorStable(),
                                ClimberCommands.setClimberPowers(0)
                        ),
                        new ParallelCommandGroup(
                                ScoreCommands.elevatorStable(),
                                ScoreCommands.moveShooterToStable(),
                                ClimberCommands.setClimberPowers(0)
                        ),
                        () -> mode == Mode.CLIMB
                )
        );
        commandXboxController.rightTrigger().onTrue(
                new ConditionalCommand(
                        ClimberCommands.setClimberPowers(-0.3),
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
                        ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.MID, 60),
                        new ConditionalCommand(
                                ScoreCommands.setShooterSpeeds(70),
                                ScoreCommands.toggleElevator(),
                                () -> mode == Mode.SHUTTLE
                        ),
                        () -> mode == Mode.SHOOTER
                )
        ).onFalse(
                new ConditionalCommand(
                        ScoreCommands.moveShooterToStable(),
                        new ParallelCommandGroup(
                                ScoreCommands.moveShooterToStable(),
                                ScoreCommands.elevatorStable()
                        ),
                        () -> mode == Mode.CLIMB
                )
        );
        commandXboxController.rightBumper().onTrue(
                new ConditionalCommand(
                        ScoreCommands.indexerFeedCommand(shooterRPS),
                        ScoreCommands.spinRollersForAmpOrTrapScore(),
                        () -> mode == Mode.SHOOTER || mode == Mode.SHUTTLE
                )
        ).onFalse(
                IntakeCommands.stopRollers()
        );


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
                            Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180.0F : 0)));

        }));
        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autos.sixPieceRed();
    }

    public void periodic() {
        double distance;
        double[] robotCoords = new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()};

        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
        else
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);

        System.out.println("Distance: " + distance);
        armAutoAimAngle = LookUpTable.findValue(distance);
        if (distance > 4.48) shooterRPS = 70;
        else shooterRPS = 60;

        System.out.println("elevator up: " + ScoreCommands.isElevatorUp);

        armAutoAim.changeSetpoint(armAutoAimAngle);
        if (armAutoAimAngle != prevArmAngle) {
            Autos.aiming.changeSetpoint(armAutoAimAngle);
            prevArmAngle = armAutoAimAngle;
        }
    }

    public void onEnable() {
        indexerSubsystem.onEnable();
        shooterSubsystem.onEnable();
        climberSubsystem.onEnable();
        noteElevator.onEnable();
    }

    public void onDisable() {
        indexerSubsystem.onDisable();
        shooterSubsystem.onDisable();
        climberSubsystem.onDisable();
        noteElevator.onEnable();

    }

    public void disabledPeriodic() {
        indexerSubsystem.disabledPeriodic();
        shooterSubsystem.disabledPeriodic();
        climberSubsystem.disabledPeriodic();
        noteElevator.disabledPeriodic();
    }

    public void simulationInit() {
        indexerSubsystem.simulationInit();
        shooterSubsystem.simulationInit();
        climberSubsystem.simulationInit();
        noteElevator.simulationInit();
    }

    public void simulationPeriodic() {
        indexerSubsystem.simulationPeriodic();
        shooterSubsystem.simulationPeriodic();
        climberSubsystem.simulationPeriodic();
        noteElevator.simulationPeriodic();
    }
}
