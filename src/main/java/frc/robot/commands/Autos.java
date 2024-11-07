package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import org.photonvision.EstimatedRobotPose;

public class Autos extends Command {

    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();
    public static PivotToCommand aiming = new PivotToCommand<>(RobotContainer.shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true);
    public static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();

    public boolean firstShot = false;
    private static Autos autos;
    public boolean part1Finished;
    public boolean part2Finished = false;
    public boolean alt1 = false;
    SwerveRequest.ApplyChassisSpeeds autonDrive = new SwerveRequest.ApplyChassisSpeeds();
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new com.pathplanner.lib.util.PIDConstants(5, 0, 0),
            new com.pathplanner.lib.util.PIDConstants(4.5, 0, 0),
            6.5, .375, new ReplanningConfig(true, true));
    private CommandSwerveDrivetrain swerveDrive;
    private Map<String, Command> commandsMap = new HashMap<>();
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();


    public Autos(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(),
                swerveDrive::seedFieldRelative, swerveDrive::getCurrentRobotChassisSpeeds,
                (speeds) -> swerveDrive.setControl(autonDrive.withSpeeds(speeds)),
                pathFollowerConfig, () -> false, swerveDrive);


        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);


//        if (Robot.estimatePose.getFirst().isPresent()) {
//            EstimatedRobotPose robotPose = Robot.estimatePose.getFirst().get();
//
//            Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();
//            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
//                    Rotation2d.fromDegrees(Robot.alliance.get() == DriverStation.Alliance.Red ? 180 : 0));
//
//            swerveDrive.addVisionMeasurement(modify, Robot.estimatePose.getSecond());
//        }

        aiming.execute();
    }

    public static Autos getInstance(CommandSwerveDrivetrain commandSwerveDriveTrain) {
        if (autos == null) autos = new Autos(commandSwerveDriveTrain);
        return autos;
    }


}