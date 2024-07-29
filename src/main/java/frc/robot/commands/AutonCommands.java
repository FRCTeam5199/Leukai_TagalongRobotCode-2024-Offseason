package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class AutonCommands extends Command {
  private static AutonCommands autonCommands;

  private AutonCommands() {}
  public AutonCommands getInstance() {
    if (autonCommands == null) { autonCommands = new AutonCommands(); }
    return autonCommands;
  }
}