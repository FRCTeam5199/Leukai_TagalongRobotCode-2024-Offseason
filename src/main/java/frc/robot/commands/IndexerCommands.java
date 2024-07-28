package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommands {
    
    public Command spitNote(IndexerSubsystem indexer){
            return new FunctionalCommand(          () -> {
                if (indexer.getAmpMode()) {
                    indexer.setRollerSpeeds(0.8, -.1, 0);
                } else {
                    indexer.setRollerSpeeds(.5, -.5, .5);
                }
            },
            () -> {
            },
            interrupted -> indexer.setRollerSpeeds(0, 0, 0),
            () -> (indexer.isNoteInIndexer() || indexer.isNoteInAmpTrap()),
            indexer
        );
    }
}
