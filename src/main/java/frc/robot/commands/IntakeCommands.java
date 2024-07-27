package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NoteElevator;

public class IntakeCommands {
    public static final IndexerSubsystem indexerSubsystem = RobotContainer.indexerSubsystem;
    public static final NoteElevator elevatorSubsystem = RobotContainer.noteElevator;

    public static Command moveElevatorDown() {
        return new FunctionalCommand(
                () -> elevatorSubsystem.goToSetpoint(0),
                () -> {
                },
                interrupted -> {
                },
                () -> false
        );
    }

    public static Command spinRollersForIntake() {
        return new FunctionalCommand(
                () -> {
                    if (indexerSubsystem.getAmpMode()) {
                        indexerSubsystem.setRollerSpeeds(0.8, 1, 0);
                    } else {
                        indexerSubsystem.setRollerSpeeds(0.8, -1, -0.05);
                    }
                },
                () -> {
                },
                interrupted -> indexerSubsystem.setRollerSpeeds(0, 0, 0),
                () -> (indexerSubsystem.isNoteInIndexer() || indexerSubsystem.isNoteInAmpTrap()),
                indexerSubsystem
        );
    }
}
