package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.ShooterSubsystem;

public class ClimberCommands {
    private static Climber climberSubsystem = Climber.getInstance();
    private static ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static AmpTrap elevatorSubsystem = AmpTrap.getInstance();
    private static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();

    public static Command moveClimbersToSetpoint(ClimberHeights leftClimberHeight, ClimberHeights rightClimberHeight) {
        return new FunctionalCommand(
                () -> climberSubsystem.setElevatorProfiles(leftClimberHeight, rightClimberHeight),
                () -> climberSubsystem.followLastElevatorProfiles(),
                interrupted -> climberSubsystem.setHoldElevatorPositions(),
                () -> climberSubsystem.isElevatorProfilesFinished(),
                climberSubsystem);
    }

    public static Command climberAscendCommand(ClimberHeights leftClimberHeight, ClimberHeights rightClimberHeight) {
        return new SequentialCommandGroup(
                new PivotToCommand<>(shooterSubsystem, ShooterPivotAngles.MAX.getRotations(), true),
                moveClimbersToSetpoint(leftClimberHeight, rightClimberHeight)
        );
    }

    public static Command climberDescendCommand(ClimberHeights leftClimberHeight, ClimberHeights rightClimberHeight) {
        return new SequentialCommandGroup(
                new PivotToCommand<>(shooterSubsystem.getPivot(), 0, true),
                moveClimbersToSetpoint(leftClimberHeight, rightClimberHeight)
        );
    }

    public static Command setClimberPowers(double percent) {
        return new FunctionalCommand(
                () -> climberSubsystem.setClimberPowers(percent),
                () -> {
                },
                interrupted -> climberSubsystem.setClimberPowers(0),
                () -> false,
                climberSubsystem
        );
    }

    public static Command moveElevatorToTrap() {
//        if (elevatorSubsystem.getElevatorSetpoint() < 0.1)
        return new ElevatorRaiseToCommand<>(elevatorSubsystem, ElevatorHeights.TRAP, true);
//        else
//            return new ElevatorRaiseToCommand<>(elevatorSubsystem, ElevatorHeights.STABLE, true);
    }
}
