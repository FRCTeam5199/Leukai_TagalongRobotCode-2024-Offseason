package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.ElevatorHeights;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.IndexerSubsystem;


public class AmpTrapCommands {
    private static AmpTrap AmpTrapInstance = AmpTrap.getInstance();
    private static IndexerSubsystem Indexer = IndexerSubsystem.getInstance();
    // AMP TRAP IS THE ELEVATOR IG
    public static Command aimElevatorStable() {
        return new ElevatorRaiseToCommand<>(AmpTrapInstance, ElevatorHeights.STABLE, true);
    }
    public static Command aimElevatorAmp() {
        return new ElevatorRaiseToCommand<>(AmpTrapInstance, ElevatorHeights.AMP, true);
    }
    public static Command aimElevatorTrap() {
        return new ElevatorRaiseToCommand<>(AmpTrapInstance, ElevatorHeights.TRAP, true);
    }
    public static Command aimAndIndexAmp() {
        return aimElevatorAmp().andThen(IntakeCommands.AmpScore());
    }
    public static Command aimAndIndexTrap() {
        return aimElevatorTrap();
    }
    public static Command resetElevatorAndIndex() {
        return aimElevatorStable().andThen(IntakeCommands.IndexerOff());
    }
    public static Command shootTrap() {
        return IntakeCommands.TrapScore();
    }
    public static Command resetShooter() {
        return new InstantCommand(() -> Indexer.setRollerSpeeds(0,0,0));
    }
}
