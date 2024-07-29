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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberHeights;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ShooterSubsystem;

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
    public static final Autos autos = new Autos(commandSwerveDrivetrain);
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

        commandXboxController.x().onTrue(ScoreCommands.ampScore());

        commandXboxController.leftTrigger().onTrue(ScoreCommands.moveShooterToAutoAim(60));
//                .onFalse(ScoreCommands.moveShooterToStable());
        commandXboxController.rightBumper().onTrue(ScoreCommands.indexerFeedCommand(60));
        commandXboxController.leftBumper().onTrue(ScoreCommands.ampScore())
                .onFalse(ScoreCommands.elevatorStable());
        commandXboxController.povLeft().onTrue(ClimberCommands.moveClimbersToSetpoint(ClimberHeights.DOWN, ClimberHeights.DOWN));
        commandXboxController.povRight().onTrue(ClimberCommands.moveClimbersToSetpoint(ClimberHeights.UP_LEFT, ClimberHeights.UP_RIGHT));


        commandXboxController.y().whileTrue(ScoreCommands.driveAutoTurn(commandXboxController, fieldCentricSwerveDrive));

        commandXboxController.button(8).onTrue(commandSwerveDrivetrain.runOnce(() -> {
            // Seed field relative pose that is alliance dependent
            var current = commandSwerveDrivetrain.getPose();
            commandSwerveDrivetrain.seedFieldRelative(
                new Pose2d(
                    current.getX(),
                    current.getY(),
                    Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180.0 : 0)));
        }));
        commandSwerveDrivetrain.registerTelemetry(logger:: telemeterize);
    }

    public Command getAutonomousCommand() {
        return autos.getBuiltAuton("6 piece red");
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
