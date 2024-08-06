package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObjectDetectionSubsystem;

public class DriveCommands {
    public RobotContainer robotContainer;
    static PIDController notePid = new PIDController(.1, .004, .00001);
    static PIDController angController = new PIDController(.1, 0.0001, 0.0001);



    public static Command goToNote(){
        return RobotContainer.commandSwerveDrivetrain.applyRequest(()-> {return new FieldCentric().withVelocityX(0).withVelocityY(notePid.calculate(RobotContainer.objectDetectionSubsystem.getNotePoseY(), -14)).withRotationalRate(angController.calculate(RobotContainer.objectDetectionSubsystem.getNotePoseX(), 0.1));});
    }

}
