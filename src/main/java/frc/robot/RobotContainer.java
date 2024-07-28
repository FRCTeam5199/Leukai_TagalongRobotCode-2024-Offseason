// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
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
  public static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  public static Shooter shooterSubsystem = Shooter.getInstance();
  public static Climber climberSubsystem = Climber.getInstance();
  public static NoteElevator noteElevator = NoteElevator.getInstance();

  public RobotContainer() {
      configureBindings();
  }

  private void configureBindings() {
    commandSwerveDrivetrain.setDefaultCommand(// Drivetrain will execute this command periodically
      new ConditionalCommand(
        commandSwerveDrivetrain.applyRequest(() -> fieldCentricSwerveDrive.withVelocityX(commandXboxController.getLeftY() * MaxSpeed) // Drive forward with
          // negative Y (forward)
          .withVelocityY(commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ),
        ScoreCommands.driveAutoTurn(commandXboxController, commandSwerveDrivetrain, fieldCentricSwerveDrive),
        () -> commandXboxController.button(7).getAsBoolean()
      )
    );

    commandXboxController.rightTrigger().onTrue(IntakeCommands.intake());

    commandXboxController.leftTrigger().onTrue(ScoreCommands.basicAutoShootCommand());

    commandXboxController.a().onTrue(IntakeCommands.switchAmpMode());
    commandXboxController.b().onTrue(IntakeCommands.switchShooterMode());

    commandXboxController.x().onTrue(ScoreCommands.ampScore());
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
