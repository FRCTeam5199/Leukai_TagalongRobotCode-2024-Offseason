package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.tagalong.PIDConstants;

public class Autos extends Command {
  private static Autos autos;
  private CommandSwerveDrivetrain swerveDrive;

  private Map<String, Command> commandsMap = new HashMap<>();
  SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
  HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new com.pathplanner.lib.util.PIDConstants(0, 0, 0), new com.pathplanner.lib.util.PIDConstants(0, 0, 0), 5.76072, .375, new ReplanningConfig());
    

  public Autos(CommandSwerveDrivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;
    AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(), swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds, (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)), pathFollowerConfig, () -> false, swerveDrive);
  }
 

  public void init() {
    initalizeCommandsMap();
    NamedCommands.registerCommands(commandsMap);
  }

  private void initalizeCommandsMap() {
    commandsMap.put("intakeCommand", IntakeCommands.intake());
    commandsMap.put("prepShooterCommand", ScoreCommands.moveShooterToAutoAim(4000));
    commandsMap.put("autoShootCommand", AutonCommands.autoShootCommand(2600));
    commandsMap.put("shootCommand", ScoreCommands.indexerFeedCommand(60));
  }

  public Command sixPieceRedAuton() {
    return AutoBuilder.buildAuto("6 piece red");
  }
}