package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Autos extends Command {
    private static Autos autos;
    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            5.76072, .375, new ReplanningConfig(true, true));
    private CommandSwerveDrivetrain swerveDrive;
    private Map<String, Command> commandsMap = new HashMap<>();
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private Autos(CommandSwerveDrivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public static Autos getInstance(CommandSwerveDrivetrain commandSwerveDriveTrain) {
    if (autos == null) autos = new Autos(commandSwerveDriveTrain);
    return autos;
  }
 

  public void init() {
    configureAutoBuilder();
    initalizeCommandsMap();
    NamedCommands.registerCommands(commandsMap);
    
    initalizeAutoChooser();
  }

  private void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, () -> false, swerveDrive);
    AutoBuilder.buildAutoChooser();
  }

   private void initalizeCommandsMap() {
       commandsMap.put("intakeCommand", IntakeCommands.intake());
       commandsMap.put("prepShooterCommand", ScoreCommands.moveShooterToAutoAim(60));
       commandsMap.put("shootCommand", ScoreCommands.indexerFeedCommand(60));
       commandsMap.put("autoShoot", ScoreCommands.moveShooterToAutoAimAndAutoShoot(60));
   }

  private void initalizeAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public Command getSelectedAuton() {
    return autoChooser.getSelected();
  }
}