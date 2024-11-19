package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class AmpTrap extends SubsystemBase {
    private static frc.robot.subsystems.AmpTrap elevatorSubsystem = frc.robot.subsystems.AmpTrap.getInstance();


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
