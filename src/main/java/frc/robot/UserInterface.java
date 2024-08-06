package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpTrapSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class UserInterface {
    private static UserInterface userInterface;
    
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
    private static final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private static final AmpTrapSubsystem ampTrapSubsystem = AmpTrapSubsystem.getInstance();

    private static final Field2d field2d = new Field2d();

    // Shuffleboard
    private static ShuffleboardTab shuffleboardTestTab = Shuffleboard.getTab("Test");
    private static GenericEntry shuffleboardEditModeComponent, shuffleboardShooterArmPositionComponent, shuffleboardShooterLeftFlywheelVelocityComponent, shuffleboardShooterRightFlywheelVelocityComponent, shuffleboardAmpTrapElevatorPositionComponent;

    private UserInterface() {
    }

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
        SmartDashboard.putData("Auto Selector", Autos.getInstance(commandSwerveDrivetrain).getAutoChooser());
        SmartDashboard.putData("Field 2D", field2d);

        shuffleboardEditModeComponent = shuffleboardTestTab.add("Edit Mode", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 0)
            .withSize(1, 1).getEntry();


        shuffleboardShooterArmPositionComponent = shuffleboardTestTab.add("Shooter Arm Position", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 0)
            .withSize(1, 1).getEntry();
        shuffleboardShooterLeftFlywheelVelocityComponent = shuffleboardTestTab.add("Shooter Left Flywheel Velocity", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 1)
            .withSize(1, 1).getEntry();
        shuffleboardShooterRightFlywheelVelocityComponent = shuffleboardTestTab.add("Shooter Right Flywheel Velocity", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(1, 1).getEntry();

        shuffleboardAmpTrapElevatorPositionComponent = shuffleboardTestTab.add("Amp/Trap Elevator Position", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(1, 1).getEntry();
    }

    public void update() {
        updateWidgets();
    }

    public void updateWidgets() {
        field2d.setRobotPose(commandSwerveDrivetrain.getPose());

        if (!shuffleboardEditModeComponent.getBoolean(false)) {
            shuffleboardShooterArmPositionComponent.setDouble(shooterSubsystem.getPivot().getPivotPosition());
            shuffleboardShooterLeftFlywheelVelocityComponent.setDouble(shooterSubsystem.getFlywheel(0).getFlywheelVelocity());
            shuffleboardShooterRightFlywheelVelocityComponent.setDouble(shooterSubsystem.getFlywheel(1).getFlywheelVelocity());
            shuffleboardAmpTrapElevatorPositionComponent.setDouble(ampTrapSubsystem.getElevator().getElevatorPosition());
        }
    }
}
