// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.generated.TunerConstants;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.LED.LEDSubsystem.LEDMode;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
    private final UserInterface userInterface = UserInterface.getInstance();
    private final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private final ApriltagSubsystem aprilTagSubsystem = new ApriltagSubsystem();
    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    public double armAngle;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        commandSwerveDrivetrain.setVisionMeasurementStdDevs(Constants.Vision.kMultiTagStdDevsAuton);
        userInterface.init();

        ledSubsystem.init();
        ledSubsystem.start();
        ledSubsystem.setMode(LEDMode.SHOOTING);

        // Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        // Logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();

        // Logger.recordOutput("Drive/Pose", commandSwerveDrivetrain.getPose());
        // Logger.recordOutput("Distance from Red Speaker", ScoreCommands.getDistance(
        //         new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()},
        //         Constants.Vision.RED_SPEAKER_COORDINATES));
        // Logger.recordOutput("Distance from Blue Speaker", ScoreCommands.getDistance(
        //         new double[]{commandSwerveDrivetrain.getPose().getX(), commandSwerveDrivetrain.getPose().getY()},
        //         Constants.Vision.BLUE_SPEAKER_COORDINATES));

        // Logger.recordOutput("Auto Aim Arm Target Angle (degrees)", RobotContainer.armAutoAimAngle);
        // Logger.recordOutput("Arm Angle (degrees)", RobotContainer.shooterSubsystem.getPivot().getPivotAbsolutePositionRot() * 360d);

        // Logger.recordOutput("Shooter Left Speed (rps)", RobotContainer.shooterSubsystem.getFlywheel().getFlywheelVelocity());
        // Logger.recordOutput("Shooter Right Speed (rps)", RobotContainer.shooterSubsystem.getFlywheel(1).getFlywheelVelocity());

        // Logger.recordOutput("Indexer Breakbeam Sensor", RobotContainer.indexerSubsystem.isNoteInIndexer());
        // Logger.recordOutput("Amp-Trap Breakbeam Sensor", RobotContainer.indexerSubsystem.isNoteInAmpTrap());
        // Logger.recordOutput("Intake Breakbeam Sensor", RobotContainer.indexerSubsystem.isNoteInIntake());

        // Logger.recordOutput("Reached Shooting Conditions", RobotContainer.shooterSubsystem.reachedShootingCondtions(RobotContainer.shooterRPS));

        // Logger.recordOutput("Bottom Intake Voltage", RobotContainer.indexerSubsystem.getRoller().getRollerMotor().getSupplyVoltage().getValueAsDouble());
        // Logger.recordOutput("Bottom Intake Current", RobotContainer.indexerSubsystem.getRoller().getRollerMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Indexer Voltage", RobotContainer.indexerSubsystem.getRoller(2).getRollerMotor().getSupplyVoltage().getValueAsDouble());
        // Logger.recordOutput("Indexer Current", RobotContainer.indexerSubsystem.getRoller(2).getRollerMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Indexer speed", RobotContainer.indexerSubsystem.getRoller(2).getRollerMotor().getVelocity().getValueAsDouble());

        // Logger.recordOutput("April Tag Target 1", aprilTagSubsystem.getTargets().get(0));

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

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.onEnable();

        commandSwerveDrivetrain.setVisionMeasurementStdDevs(Constants.Vision.kMultiTagStdDevsAuton);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }


    }

    @Override
    public void autonomousPeriodic() {
        Pair<Optional<EstimatedRobotPose>, Double> estimatePose = aprilTagSubsystem.getEstimatedGlobalPose();
        Pair<Optional<EstimatedRobotPose>, Double> estimatePoseBack = aprilTagSubsystem.getEstimatedGlobalPose();

        if (estimatePose.getFirst().isPresent() && estimatePoseBack.getFirst().isPresent()) {
            EstimatedRobotPose robotPose = aprilTagSubsystem.getAmbiguity() < aprilTagSubsystem.getAmbiguityBack()
                    ? estimatePose.getFirst().get() : estimatePoseBack.getFirst().get();

            Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();
            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
                    Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0));

            commandSwerveDrivetrain.addVisionMeasurement(modify, estimatePose.getSecond());
        } else if (estimatePose.getFirst().isPresent()) {
            EstimatedRobotPose robotPose = estimatePose.getFirst().get();

            Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();
            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
                    Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0));

            commandSwerveDrivetrain.addVisionMeasurement(modify, estimatePose.getSecond());
        } else if (estimatePoseBack.getFirst().isPresent()) {
            EstimatedRobotPose robotPose = estimatePoseBack.getFirst().get();
            Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();

            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
                    Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0));

            commandSwerveDrivetrain.addVisionMeasurement(modify, estimatePose.getSecond());
        }

//        Autos.aimingWhileMoving.initialize();
//        Autos.aimingWhileMoving.execute();
//        Autos.aimingWhileMoving.end(Autos.aimingWhileMoving.isFinished());

        Autos.aiming.execute();
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
        Pair<Optional<EstimatedRobotPose>, Double> estimatePose1 = aprilTagSubsystem.getEstimatedGlobalPose();

        if (estimatePose1.getFirst().isPresent()) {
            EstimatedRobotPose robotPose = estimatePose1.getFirst().get();

            Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();

            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(), commandSwerveDrivetrain.getPose().getRotation());

            commandSwerveDrivetrain.addVisionMeasurement(modify, estimatePose1.getSecond());
        }

        RobotContainer.teleopPeriodic();
        commandSwerveDrivetrain.getPigeon2().getAngle();
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
