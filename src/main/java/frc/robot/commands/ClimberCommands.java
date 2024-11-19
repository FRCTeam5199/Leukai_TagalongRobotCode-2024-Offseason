package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ClimberCommands {

    Climber climber;

    public ClimberCommands() {

    }

    public Command climbUp(){
        return this.climber.runOnce(() -> climber.setClimberPowers(50));
    }

    public Command climbDown() {
        return this.climber.runOnce(() -> climber.setClimberPowers(-15));
    }

    public Command climbStop() {
        return this.climber.runOnce(() -> climber.setClimberPowers(0));
    }

}
