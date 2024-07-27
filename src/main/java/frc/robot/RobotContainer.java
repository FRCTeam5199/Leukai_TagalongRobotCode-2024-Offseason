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
import frc.robot.commands.base.AutoShootCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  CommandXboxController commandXboxController = new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);
  
  public final static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain; // My drivetrain
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // The robot's subsystems and commands are defined here...
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric fieldCentricSwerveDrive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final PIDController driveRotationalPIDController = new PIDController(0.05, 0, 0);

  IndexerSubsystem indexerSubsystem = new IndexerSubsystem("configs/indexer/indexerConf.json");
  Shooter shooterSubsystem = new Shooter("configs/shooter/shooterConf.json");
  // NoteElevator noteElevator = new NoteElevator("configs/notevator/notevatorConf.json");
  Climber climber = new Climber("configs/climber/climberConf.json");

  // Commands
  AutoShootCommands autoShootCommands = new AutoShootCommands();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
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

    commandXboxController.leftTrigger().onTrue(autoShootCommands.BasicAutoShootCommand(shooterSubsystem, indexerSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void onEnable() {
    indexerSubsystem.onEnable();
    shooterSubsystem.onEnable();
  }

  public void onDisable() {
    indexerSubsystem.onDisable();
    shooterSubsystem.onDisable();
  }

  public void disabledPeriodic() {
    indexerSubsystem.disabledPeriodic();
    shooterSubsystem.disabledPeriodic();
  }

  public void simulationInit() {
    indexerSubsystem.simulationInit();
    shooterSubsystem.simulationInit();
  }

  public void simulationPeriodic() {
    indexerSubsystem.simulationPeriodic();
    shooterSubsystem.simulationPeriodic();
  }
}
