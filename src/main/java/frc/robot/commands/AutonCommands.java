package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.LookUpTable;

public class AutonCommands {
    private static AutonCommands autonCommands;

    private static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();

    private AutonCommands() {
    }

    public AutonCommands getInstance() {
        if (autonCommands == null) {
            autonCommands = new AutonCommands();
        }
        return autonCommands;
    }


    public static Command noteCheck1(){
        if(objectDetection.notePresent() == false){

            return new SequentialCommandGroup(AutoBuilder.buildAuto("6 Piece Red alt 1"));

        }else{
            return new InstantCommand(()-> System.out.println("There is a note"));

        }
    }
}