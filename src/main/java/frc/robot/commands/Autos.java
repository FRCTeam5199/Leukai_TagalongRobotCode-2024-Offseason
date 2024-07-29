package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class Autos extends Command {
  private static Autos autos;

  private Map<String, Command> commandsMap = new HashMap<>();

  private Autos() {}
  public static Autos getInstance() {
    if (autos == null) { autos = new Autos(); }
    return autos;
  }

  public void init() {
    initalizeCommandsMap();
    NamedCommands.registerCommands(commandsMap);
  }

  private void initalizeCommandsMap() {
    commandsMap.put("intakeCommand", IntakeCommands.intake());
    commandsMap.put("prepShooterCommand", ScoreCommands.moveShooterToAutoAim(4000));
    commandsMap.put("autoShootCommand", AutonCommands.autoShootCommand(2600));
    commandsMap.put("shootCommand", ScoreCommands.indexerFeedCommand());
  }

  public String getSelectedPath() {
    return "6-Piece Red Side Path";
  }
}