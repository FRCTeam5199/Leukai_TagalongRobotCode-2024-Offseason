package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.base.ClimberHeights;
import frc.robot.subsystems.AmpTrap;
import frc.robot.constants.Constants;


public class IntakeCommands {
    /**
     * Creates a new IntakeCommands.
     */
    private static AmpTrap elevatorSubsystem = AmpTrap.getInstance();


    public static Command MoveElevatorToShoot() {
        return new FunctionalCommand(
                () -> {
                },
                () -> elevatorSubsystem.getElevator().setElevatorHeight(Constants.Amp.AMP_TOP_POSITiON),
                interrupted -> elevatorSubsystem.getElevator().setHoldElevatorPosition(true),
                () -> false,
                elevatorSubsystem
        );
    }

    public static Command MoveElevatorBack() {
        return new FunctionalCommand(
                () -> {
                },
                () -> elevatorSubsystem.getElevator().setElevatorHeight(Constants.Amp.AMP_START_POSITiON),
                interrupted -> elevatorSubsystem.getElevator().setHoldElevatorPosition(true),
                () -> false,
                elevatorSubsystem
        );
    }

}

