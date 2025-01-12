package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.RobotContainer;


public class ClimberCommands {

    static Climber climber;



    public ClimberCommands() {
        climber = Climber.getInstance();
    }

    public static Command climbUp(){
        return new InstantCommand(() -> climber.setClimberPowers(-.2));
    }

    public static Command climbDown() {
        return new InstantCommand(() -> climber.setClimberPowers(.2));
    }
    public static Command climbStop() {
        return new InstantCommand(() -> climber.setClimberPowers(0));
    }

}