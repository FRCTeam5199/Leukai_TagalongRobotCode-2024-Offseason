package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpTrapSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.tagalong.TagalongAngle;

public class UserInterface {
    private static UserInterface userInterface;

    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static final AmpTrapSubsystem ampTrapSubsystem = AmpTrapSubsystem.getInstance();

    private static final Field2d field2d = new Field2d();

    // Shuffleboard
    private static ShuffleboardTab shuffleboardRobotTab = Shuffleboard.getTab("Robot");
    private static ShuffleboardTab shuffleboardSubsystemsTab = Shuffleboard.getTab("Subsystems");
    private static ShuffleboardTab shuffleboardPreGameTab = Shuffleboard.getTab("Pre-Game");
    private static ShuffleboardTab shuffleboardGameTab = Shuffleboard.getTab("Game");

    private static GenericEntry shuffleboardGyroComponent, shuffleboardDrivebaseComponent;
    private static GenericEntry shuffleboardEditModeComponent, shuffleboardShooterArmPositionComponent, shuffleboardShooterLeftFlywheelVelocityComponent, shuffleboardShooterRightFlywheelVelocityComponent, shuffleboardShooterRightFlywheelFeedForwardComponent, shuffleboardShooterLeftFlywheelFeedForwardComponent, shuffleboardAmpTrapElevatorPositionComponent;
    // private static GenericEntry shuffleboardEditModeComponent, shuffleboardShooterArmPositionComponent, shuffleboardShooterLeftFlywheelVelocityComponent, shuffleboardShooterRightFlywheelVelocityComponent, shuffleboardAmpTrapElevatorPositionComponent;
    // private static GenericEntry shuffleboardEditModeComponent, shuffleboardShooterArmPositionComponent, shuffleboardShooterLeftFlywheelVelocityComponent, shuffleboardShooterRightFlywheelVelocityComponent, shuffleboardAmpTrapElevatorPositionComponent;

    private UserInterface() {}

    public static UserInterface getInstance() {
        if (userInterface == null) userInterface = new UserInterface();
        return userInterface;
    }

    public float getShooterPositionComponentData() {
        if (shuffleboardShooterArmPositionComponent != null) {
            return shuffleboardShooterArmPositionComponent.getFloat(0);
        }
        return 0;
    }

    public void init() {
        // initalizeData();
        initalizeWidgets();
    }

    // private void initalizeData() {}

    private void initalizeWidgets() {
        // SmartDashboard.putData("Auto Selector", Autos.getInstance(commandSwerveDrivetrain).getAutoChooser());
        // SmartDashboard.putData("Field 2D", field2d);

        // Drive Tab
        shuffleboardGyroComponent = shuffleboardRobotTab.add("Gyro", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 0)
            .withSize(2, 2).getEntry();
        shuffleboardDrivebaseComponent = shuffleboardRobotTab.add("Drivebase", commandSwerveDrivetrain)
            .withWidget(BuiltInWidgets.kMecanumDrive)
            .withPosition(0, 0)
            .withSize(2, 2).getEntry();

        // Subsystems Tab
        shuffleboardEditModeComponent = shuffleboardSubsystemsTab.add("Edit Mode", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 0)
            .withSize(1, 1).getEntry();


        shuffleboardShooterArmPositionComponent = shuffleboardSubsystemsTab.add("Shooter Arm Position", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 0)
            .withSize(1, 1).getEntry();
        shuffleboardShooterLeftFlywheelVelocityComponent = shuffleboardSubsystemsTab.add("Shooter Left Flywheel Velocity", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 1)
            .withSize(1, 1).getEntry();
        shuffleboardShooterRightFlywheelVelocityComponent = shuffleboardSubsystemsTab.add("Shooter Right Flywheel Velocity", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 2)
            .withSize(1, 1).getEntry();
        shuffleboardShooterRightFlywheelFeedForwardComponent = shuffleboardSubsystemsTab.add("Shooter Left Flywheel Feed Forward Enabled", 0)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 1)
            .withSize(1, 1).getEntry();
        shuffleboardShooterRightFlywheelFeedForwardComponent = shuffleboardSubsystemsTab.add("Shooter Left Flywheel Feed Forward Enabled", 0)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 2)
            .withSize(1, 1).getEntry();

        shuffleboardAmpTrapElevatorPositionComponent = shuffleboardSubsystemsTab.add("Amp/Trap Elevator Position", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3, 0)
            .withSize(1, 1).getEntry();
    }

    public void update() {
        updateWidgets();
    }

    public void updateWidgets() {
        field2d.setRobotPose(commandSwerveDrivetrain.getPose());

        // Drive Tab
        shuffleboardGyroComponent.setValue(commandSwerveDrivetrain.getPigeon2());
        shuffleboardDrivebaseComponent.setValue(commandSwerveDrivetrain);

        // Subsystems Tab
        if (!shuffleboardEditModeComponent.getBoolean(false)) {
            shuffleboardShooterArmPositionComponent.setDouble(shooterSubsystem.getPivot().getPivotPosition());
            shuffleboardShooterLeftFlywheelVelocityComponent.setDouble(shooterSubsystem.getFlywheel(0).getFlywheelVelocity());
            shuffleboardShooterRightFlywheelVelocityComponent.setDouble(shooterSubsystem.getFlywheel(1).getFlywheelVelocity());
            shuffleboardShooterLeftFlywheelFeedForwardComponent.setDouble(shooterSubsystem.getFlywheel(0).getFlywheelVelocity());
            shuffleboardShooterRightFlywheelFeedForwardComponent.setDouble(shooterSubsystem.getFlywheel(1).getFlywheelVelocity());

            shuffleboardAmpTrapElevatorPositionComponent.setDouble(ampTrapSubsystem.getElevator().getElevatorHeightM());
        } else {
            shooterSubsystem.getPivot().setPivotProfile(shuffleboardShooterArmPositionComponent.getDouble(shooterSubsystem.getPivot().getPivotPosition()));
            // shooterSubsystem.getFlywheel(0).setFlywheelControl(shuffleboardShooterLeftFlywheelVelocityComponent.getDouble(shooterSubsystem.getFlywheel(0).getFlywheelVelocity()), shuffleboardShooterLeftFlywheelFeedForwardComponent.getBoolean(false));
            // shooterSubsystem.getFlywheel(1).setFlywheelControl(shuffleboardShooterRightFlywheelVelocityComponent.getDouble(shooterSubsystem.getFlywheel(1).getFlywheelVelocity()), shuffleboardShooterRightFlywheelFeedForwardComponent.getBoolean(false));

            ampTrapSubsystem.getElevator().setElevatorHeight(shuffleboardAmpTrapElevatorPositionComponent.getDouble(ampTrapSubsystem.getElevator().getElevatorHeightM()));
        }
    }
}
