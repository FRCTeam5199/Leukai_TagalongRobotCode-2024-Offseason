package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;

public class TrapCommands{
    
    public Command trapPrep(NoteElevator noteElevator,Shooter shooter, Climber climber){
        return new SequentialCommandGroup(
            new ElevatorRaiseToCommand<>(noteElevator, ()-> 10),
            new ClimberCommands().ClimberAscendCommand(shooter, climber, 10)
        );
    }

    public Command trapClimb(IndexerSubsystem indexer, Shooter shooter, Climber climber){
        return new SequentialCommandGroup(
            new ClimberCommands().ClimberDescendCommand(shooter, climber, 0),
            new IndexerCommands().spitNote(indexer)
        );
    }


}
