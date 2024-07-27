package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.Shooter;
import frc.robot.tagalong.TagalongAngle;

public class ClimberCommands extends Command {
    RobotContainer robotContainer;

    public Command ClimberAscendCommand(Shooter shooter, Climber climber){
        return new SequentialCommandGroup(
            new PivotToCommand<>(shooter.getPivot(), 30, true),
            new ParallelCommandGroup(
                new ElevatorRaiseToCommand<>(0, climber, ()-> 12),
                new ElevatorRaiseToCommand<>(1, climber, ()-> 12)
            )
        );
    }
    
    public Command ClimberDescendCommand(Shooter shooter, Climber climber){
        return new SequentialCommandGroup(
            new PivotToCommand<>(shooter.getPivot(), 0, true),
            new ParallelCommandGroup(
                new ElevatorRaiseToCommand<>(0, climber, ()-> 0),
                new ElevatorRaiseToCommand<>(1, climber, ()-> 0)
            )
        );
    }
}
