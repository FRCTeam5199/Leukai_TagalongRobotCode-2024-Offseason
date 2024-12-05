// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utility.Mode.AMP;
import static frc.robot.utility.Mode.CLIMB;
import static frc.robot.utility.Mode.SHOOTER;
import static frc.robot.utility.Mode.SHUTTLE;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AmpTrapCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.Mode;
import frc.robot.utility.hii;

public class RobotContainer {
    public static final CommandXboxController commandXboxController = new CommandXboxController(
            Ports.DRIVER_XBOX_USB_PORT);
    public final static CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain; // My drivetrain
    public static final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    public static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    public static final Climber climberSubsystem = Climber.getInstance();
    public static final ClimberCommands climber = new ClimberCommands();
    public static final IntakeCommands intaketh = new IntakeCommands();
    public static final IntakeCommands intake = new IntakeCommands();
    public static final AmpTrap ampTrap = AmpTrap.getInstance();
    public final static ObjectDetectionSubsystem objectDetection = ObjectDetectionSubsystem.getInstance();
    //     public static final Autos autos = new Autos(commandSwerveDrivetrain);
    // driving in open loop
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public static final hii orchestraPlayer = new hii();
    public static double armAutoAimAngle;
    public static Mode mode = Mode.SHOOTER;
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

        Shuffleboard.getTab("Drive Info").addBoolean("Climb Mode", () -> mode == CLIMB);
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
        commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                commandSwerveDrivetrain.applyRequest(
                        () -> {
                            return
                                    // Drive forward with negative Y (forward)
                                    fieldCentricSwerveDrive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed)
                                            // Drive left with negative X (left)
                                            .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed)
                                            // Drive counterclockwise with negative X (left)
                                            .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate);
                        }
                )
        );

        commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
        commandXboxController.x().onTrue((new InstantCommand(() -> setMode(CLIMB)))
                .andThen(new InstantCommand(() -> System.out.println("It's Climbing Time")))
                .andThen((ShooterCommands.aimShooterClimb())));
        commandXboxController.a().onTrue((new InstantCommand(() -> setMode(AMP)))
                .andThen(new InstantCommand(() -> System.out.println("It's Amping Time"))));
        commandXboxController.b().onTrue((new InstantCommand(() -> setMode(SHOOTER)))
                .andThen(new InstantCommand(() -> System.out.println("It's Shooting Time"))));
        commandXboxController.y().onTrue((new InstantCommand(() -> setMode(SHUTTLE)))
                .andThen(new InstantCommand(() -> System.out.println("It's Shuttling Time!"))));

        // orchestra commands
        commandXboxController.povDown().onTrue((new InstantCommand(hii::playMegolovania))
                .andThen(new InstantCommand(() -> System.out.println("Playing Megolovania"))));
        commandXboxController.povLeft().onTrue((new InstantCommand(hii::playTotikfr))
                .andThen(new InstantCommand(() -> System.out.println("Playing Totikfr"))));
        commandXboxController.povRight().onTrue((new InstantCommand(hii::playCarelessWhisper))
                .andThen(new InstantCommand(() -> System.out.println("Playing CarelessWhisper"))));

        //CLIMB MODE

        //Climb up
        commandXboxController.rightTrigger()
                .onTrue(climber.climbUp()
                        .andThen(new InstantCommand(() -> System.out.println("Climb Up"))
                                .onlyIf(() -> getMode() == CLIMB)));
        commandXboxController.rightTrigger()
                .onFalse(climber.climbStop()
                        .andThen(new InstantCommand(() -> System.out.println("Climb Stop"))
                                .onlyIf(() -> getMode() == CLIMB)));

        //Climb down
        commandXboxController.leftTrigger()
                .onTrue(climber.climbDown()
                        .andThen(new InstantCommand(() -> System.out.println("Climb Down"))
                                .onlyIf(() -> getMode() == CLIMB)));
        commandXboxController.leftTrigger()
                .onFalse(climber.climbStop()
                        .andThen(new InstantCommand(() -> System.out.println("Climb Stop"))
                                .onlyIf(() -> getMode() == CLIMB)));

        //Shoot amp
        commandXboxController.rightBumper();

        //Elevator
        commandXboxController.leftBumper()
                .onTrue(AmpTrapCommands.aimAndIndexTrap())
                .onFalse(AmpTrapCommands.resetElevatorAndIndex());


        // Index to amp
        commandXboxController.rightTrigger()
                .onTrue(IntakeCommands.IntakeToAmp())
                .onFalse(IntakeCommands.IndexerOff())
        ;
        // Index/Intake  to shooter
        commandXboxController.leftTrigger()
                .onTrue(IntakeCommands.IntakeToShooter())
                .onFalse(IntakeCommands.IndexerOff())
        ;

        commandXboxController.button(8).onTrue(commandSwerveDrivetrain.runOnce(() -> {
            // Seed field relative pose that is alliance dependent
            var current = commandSwerveDrivetrain.getPose();
            commandSwerveDrivetrain.seedFieldRelative(
                    new Pose2d(
                            current.getX(),
                            current.getY(),
                            Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180.0d : 0)));
//            commandSwerveDrivetrain.getPigeon2().setYaw(new Pose2d(
//                    current.getX(),
//                    current.getY(),
//                    Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180.0d : 0)).getRotation().getDegrees());

        }));
    }

    public Command getAutonomousCommand() {
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        //     return Autos.autonChooserRed.getSelected();
        // } else {
        //     return Autos.autonChooserBlue.getSelected();
        // }
        return null;
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
