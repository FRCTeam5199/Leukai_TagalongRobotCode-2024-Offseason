// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.LookUpTable;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(
            Ports.DRIVER_XBOX_USB_PORT);

    public final static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain; // My drivetrain
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final Climber climberSubsystem = Climber.getInstance();
    public static final AmpTrap ampTrap = AmpTrap.getInstance();
    public final static  ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();
    public static final Autos autos = Autos.getInstance(commandSwerveDrivetrain);
    // driving in open loop
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
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

    public static void getAngle() {
        double distance;
        double[] robotCoords = new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()};
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
        else
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);
        ScoreCommands.armAutoAimAngle = LookUpTable.findValue(distance);
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


        commandXboxController.rightTrigger().onTrue(IntakeCommands.intake());
        commandXboxController.a().onTrue(IntakeCommands.switchAmpMode());
        commandXboxController.b().onTrue(IntakeCommands.switchShooterMode());
//        commandXboxController.povUp().whileTrue(DriveCommands.goToNote());

        commandXboxController.x().onTrue(ClimberCommands.scoreTrap());
        commandXboxController.povUp().onTrue(ClimberCommands.toggleElevatorTrap());
        commandXboxController.povDown().onTrue(ScoreCommands.elevatorStable());

//        commandXboxController.leftTrigger().onTrue(ScoreCommands.moveShooterToSetpointAndSpeed(ShooterPivotAngles.MID, 60))
//                .onFalse(ScoreCommands.moveShooterToStable());
        commandXboxController.y().onTrue(ScoreCommands.trapCommand());

        commandXboxController.leftTrigger().whileTrue(new ParallelCommandGroup(
                ScoreCommands.driveAutoTurn(commandXboxController,
                        fieldCentricSwerveDrive),
                new SequentialCommandGroup(
                        new InstantCommand(RobotContainer::getAngle),
                        ScoreCommands.moveShooterToAutoAim(60)
                )
        )).onFalse(ScoreCommands.moveShooterToStable());

//                .onFalse(ScoreCommands.moveShooterToStable());
        commandXboxController.rightBumper().onTrue(ScoreCommands.indexerFeedCommand(60));
        commandXboxController.leftBumper().onTrue(ScoreCommands.ampScore())
                .onFalse(ScoreCommands.elevatorStable());
        commandXboxController.povLeft().onTrue(ClimberCommands.moveClimbersToSetpoint(ClimberHeights.DOWN, ClimberHeights.DOWN));
        commandXboxController.povRight().onTrue(ClimberCommands.moveClimbersToSetpoint(ClimberHeights.UP_LEFT, ClimberHeights.UP_RIGHT));

        commandXboxController.povDown().whileTrue(ScoreCommands.driveAutoTurn(commandXboxController, fieldCentricSwerveDrive));

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
        return autos.sixPieceRedwithAlt();
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
