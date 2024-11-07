package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.commands.base.ClimberHeights;
import frc.robot.commands.base.ElevatorHeights;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.ShooterSubsystem;

public class TrapCommands {
    public static AmpTrap noteElevator;
    public static ShooterSubsystem shooter;
    public static Climber climber;


}
