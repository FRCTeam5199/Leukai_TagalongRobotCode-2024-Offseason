// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.base.ClimberCommands;
import frc.robot.commands.base.ElevatorHeights;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.utility.LookUpTable;
import frc.robot.utility.Mode;

import java.util.Map;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(
            Ports.DRIVER_XBOX_USB_PORT);
    public final static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain; // My drivetrain
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final Climber climberSubsystem = Climber.getInstance();
    public static final AmpTrap ampTrap = AmpTrap.getInstance();
    public final static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();
    public static final Autos autos = new Autos(commandSwerveDrivetrain);
    // driving in open loop
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static double armAutoAimAngle;
    private static Mode mode = Mode.SHOOTER;
    public double prevArmAngle = 0;

    // The robot's subsystems and commands are defined here...
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private double MaxAngularRate = 2.5 * Math.PI;
    private final SwerveRequest.FieldCentric fieldCentricSwerveDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
    private static boolean isIdling = false;
    private static boolean isShooting = false;
    private static boolean hasUpdatedAutoAimShot = false;
    private static Timer timer;
    private double distance;
    private double angleOffset;
    public static double driveAngleOffset;


    public RobotContainer() {


        Shuffleboard.getTab("Shooter Tuning").addNumber("Distance", () -> distance);
        Shuffleboard.getTab("Shooter Tuning").addNumber("Target Angle", () -> armAutoAimAngle);
        Shuffleboard.getTab("Shooter Tuning").addNumber("Current Angle", () -> shooterSubsystem.getPivot().getPivotAbsolutePositionRot() * 360d);
        Shuffleboard.getTab("Shooter Tuning").addNumber("Shooter 1 Speed", () -> shooterSubsystem.getFlywheel(0).getFlywheelVelocity());
        Shuffleboard.getTab("Shooter Tuning").addNumber("Shooter 2 Speed", () -> shooterSubsystem.getFlywheel(1).getFlywheelVelocity());

        Shuffleboard.getTab("Drive Info").addBoolean("Climb Mode", () -> mode == Mode.CLIMB);
        Shuffleboard.getTab("Drive Info").addBoolean("Shoot Mode", () -> mode == Mode.SHOOTER);
        Shuffleboard.getTab("Drive Info").addBoolean("Shuttle Mode", () -> mode == Mode.SHUTTLE);

        angleOffset = UserInterface.getInstance().getShooterOffset();
        timer = new Timer();
        timer.restart();

        configureBindings();
    }

    public static Mode getMode() {
        return mode;
    }

    public static void setMode(Mode mode) {
        RobotContainer.mode = mode;
    }

    private void configureBindings() {
        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Autos.autonChooserRed.getSelected();
        } else {
            return Autos.autonChooserBlue.getSelected();
        }
    }

    public void periodic() {
    }

    public static void teleopPeriodic() {

    }

    public void onEnable() {
        indexerSubsystem.onEnable();
        shooterSubsystem.onEnable();
        climberSubsystem.onEnable();
        ampTrap.onEnable();
    }

    public void onDisable() {
        indexerSubsystem.onDisable();
        shooterSubsystem.onDisable();
        climberSubsystem.onDisable();
        ampTrap.onEnable();

    }

    public void disabledPeriodic() {
        indexerSubsystem.disabledPeriodic();
        shooterSubsystem.disabledPeriodic();
        climberSubsystem.disabledPeriodic();
        ampTrap.disabledPeriodic();
    }

    public void simulationInit() {
        indexerSubsystem.simulationInit();
        shooterSubsystem.simulationInit();
        climberSubsystem.simulationInit();
        ampTrap.simulationInit();
    }

    public void simulationPeriodic() {
        indexerSubsystem.simulationPeriodic();
        shooterSubsystem.simulationPeriodic();
        climberSubsystem.simulationPeriodic();
        ampTrap.simulationPeriodic();
    }
}
