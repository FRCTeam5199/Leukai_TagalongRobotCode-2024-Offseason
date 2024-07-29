package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimberHeights;
import frc.robot.commands.ElevatorHeights;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.NoteElevator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.tagalong.TagalongAngle;

public class ClimberCommands {
    private static Climber climberSubsystem = Climber.getInstance();
    private static ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public static Command moveClimbersToSetpoint(ClimberHeights leftClimberHeight, ClimberHeights rightClimberHeight) {
        return new FunctionalCommand(
                () -> climberSubsystem.setElevatorProfiles(leftClimberHeight, rightClimberHeight),
                () -> climberSubsystem.followLastElevatorProfiles(), 
                interrupted -> climberSubsystem.setHoldElevatorPositions(),
                () -> climberSubsystem.isElevatorProfilesFinished(),
                climberSubsystem);
    }

    public Command climberAscendCommand(ClimberHeights leftClimberHeight, ClimberHeights rightClimberHeight) {
        return new SequentialCommandGroup(
                new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.MAX, true),
                moveClimbersToSetpoint(leftClimberHeight, rightClimberHeight)
        );
    }

    public Command climberDescendCommand(ClimberHeights leftClimberHeight, ClimberHeights rightClimberHeight) {
        return new SequentialCommandGroup(
                new PivotToCommand<>(shooterSubsystem.getPivot(), 0, true),
                moveClimbersToSetpoint(leftClimberHeight, rightClimberHeight)
        );
    }
}
