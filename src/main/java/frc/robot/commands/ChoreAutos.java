package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ChoreAutos{

    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();

    ChoreoTrajectory traj = Choreo.getTrajectory("Trajectory"); 
    ChoreoControlFunction control;
    public CommandSwerveDrivetrain drivetrain;
    boolean mirror;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();


    public ChoreAutos(CommandSwerveDrivetrain swerveDrive){
        drivetrain = swerveDrive;
    }

    public Command choreoAutoCommand(){
        return Choreo.choreoSwerveCommand(
            traj, // 
            ()-> drivetrain.getPose(), // 
            new PIDController(1, 0.0, 0.0), // 
            new PIDController(1, 0.0, 0.0), // 
            new PIDController(1, 0.0, 0.0), // 
            (ChassisSpeeds speeds) -> drivetrain.setControl(autonDrive.withSpeeds(speeds)),
            () ->  mirror = alliance.isPresent() && alliance.get() == Alliance.Red,
            drivetrain 
        );
    }

}
