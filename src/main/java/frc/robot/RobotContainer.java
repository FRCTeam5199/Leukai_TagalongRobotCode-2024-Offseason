// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorHeights;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.commands.TrapCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.TrapCommands;

public class RobotContainer {
    public final static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain; // My drivetrain
    // NoteElevator noteElevator = new NoteElevator("configs/notevator/notevatorConf.json");
    public static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static Shooter shooterSubsystem = Shooter.getInstance();
    public static Climber climberSubsystem = Climber.getInstance();
    public static NoteElevator noteElevator = NoteElevator.getInstance();
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final PIDController driveRotationalPIDController = new PIDController(0.05, 0, 0);
    CommandXboxController commandXboxController = new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);
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
        commandXboxController.y().onTrue(TrapCommands.trapPrep());

        commandXboxController.rightTrigger().onTrue(IntakeCommands.intake());
        commandXboxController.a().onTrue(IntakeCommands.switchAmpMode());
        commandXboxController.b().onTrue(IntakeCommands.switchShooterMode());

        commandXboxController.x().onTrue(ScoreCommands.ampScore());

//        commandXboxController.leftTrigger().onTrue(ScoreCommands.basicAutoShootCommand());
        commandXboxController.povDown().onTrue(IntakeCommands.setShooterPivotToSetpoint(ShooterPivotAngles.MID));
        commandXboxController.povLeft().onTrue(ScoreCommands.moveElevatorToSetpoint(ElevatorHeights.AMP));
        commandXboxController.povRight().onTrue(ScoreCommands.moveElevatorToSetpoint(ElevatorHeights.TRAP));

        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(() -> fieldCentricSwerveDrive.withVelocityX(commandXboxController.getLeftY() * MaxSpeed) // Drive forward with
                        // negative Y (forward)
                        .withVelocityY(commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        commandXboxController.button(7).onTrue(
                new ConditionalCommand(
                        new InstantCommand(() -> commandSwerveDrivetrain.applyRequest(
                                () -> fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY()).withVelocityY(-commandXboxController.getLeftX())
                                        .withRotationalRate(driveRotationalPIDController.calculate(commandSwerveDrivetrain.getPose().getRotation().getDegrees(), Units.radiansToDegrees(Math.atan((5.59 - commandSwerveDrivetrain.getPose().getY()) / (16.58 - commandSwerveDrivetrain.getPose().getX()))))))),
                        new InstantCommand(() -> commandSwerveDrivetrain.applyRequest(
                                () -> fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY()).withVelocityY(-commandXboxController.getLeftX())
                                        .withRotationalRate(driveRotationalPIDController.calculate((commandSwerveDrivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees()), Units.radiansToDegrees(Math.atan((5.48 - commandSwerveDrivetrain.getPose().getY()) / (-0.0381 - commandSwerveDrivetrain.getPose().getX()))))))),
                        () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ));

        commandXboxController.button(8).onTrue(commandSwerveDrivetrain.runOnce(() -> commandSwerveDrivetrain.seedFieldRelative()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
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
