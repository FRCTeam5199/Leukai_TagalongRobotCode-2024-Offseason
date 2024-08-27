// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.commands.base.PivotToCommand;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.LookUpTable;

public class Robot extends TimedRobot {
    private final UserInterface userInterface = UserInterface.getInstance();
    private final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private final ApriltagSubsystem aprilTagSubsystem = new ApriltagSubsystem();
    public double armAngle;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private PivotToCommand aim = new PivotToCommand<>(RobotContainer.shooterSubsystem, ShooterPivotAngles.STABLE.getRotations(), true);

    @Override
    public void robotInit() {

        m_robotContainer = new RobotContainer();
        commandSwerveDrivetrain.setVisionMeasurementStdDevs(Constants.Vision.kMultiTagStdDevsAuton);
        userInterface.init();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.onDisable();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        // This corresponds to the field truth, which is always the blue perspective
        commandSwerveDrivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0));

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.onEnable();

        commandSwerveDrivetrain.setVisionMeasurementStdDevs(Constants.Vision.kMultiTagStdDevsAuton);


        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        Optional<EstimatedRobotPose> estimatePose1 = aprilTagSubsystem.getEstimatedGlobalPose();

        if (estimatePose1.isPresent()) {

            EstimatedRobotPose robotPose = estimatePose1.get();

            Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();

            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
                    Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0));

            commandSwerveDrivetrain.addVisionMeasurement(modify, aprilTagSubsystem.getTimestamp());
        }

        double distance;
        double[] robotCoords = new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()};
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.RED_SPEAKER_COORDINATES);
        else
            distance = ScoreCommands.getDistance(robotCoords, Constants.Vision.BLUE_SPEAKER_COORDINATES);

        armAngle = LookUpTable.findValue(distance);


    }


    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.onEnable();


        commandSwerveDrivetrain.setVisionMeasurementStdDevs(Constants.Vision.kMultiTagStdDevsTeleop);


        // This corresponds to what direction the driver is facing at a given time
        commandSwerveDrivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0));
    }

    @Override
    public void teleopPeriodic() {
        Optional<EstimatedRobotPose> estimatePose1 = aprilTagSubsystem.getEstimatedGlobalPose();

        if (estimatePose1.isPresent()) {

            EstimatedRobotPose robotPose = estimatePose1.get();

            Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();

            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(), commandSwerveDrivetrain.getPose().getRotation());

            commandSwerveDrivetrain.addVisionMeasurement(modify, aprilTagSubsystem.getTimestamp());
        }
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationInit() {
        m_robotContainer.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        m_robotContainer.simulationPeriodic();
    }
}
