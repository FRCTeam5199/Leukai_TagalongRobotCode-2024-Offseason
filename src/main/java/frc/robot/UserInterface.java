package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class UserInterface {
    private static UserInterface userInterface;

    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;

    private static final Field2d field2d = new Field2d();
    private static List<Translation2d> robotTranslation2ds;

    private UserInterface() {}
    public static UserInterface getInstance() {
        if (userInterface == null) userInterface = new UserInterface();
        return userInterface;
    }

    public void init() {
        initalizeWidgets();
    }
    private void initalizeWidgets() {
        SmartDashboard.putData("Auto Selector", Autos.getInstance(commandSwerveDrivetrain).getAutoChooser());
        SmartDashboard.putData("Field 2D", field2d);
    }

    public void update() {
        updateWidgets();
    }

    public void updateWidgets() {
        field2d.setRobotPose(commandSwerveDrivetrain.getPose());
        // field2d.getObject("traj").setTrajectory(generateRobotTrajectory());
    }

    // private Trajectory generateRobotTrajectory() {
    //     if (robotTranslation2ds.size() > 30) robotTranslation2ds.remove(0);
    //     robotTranslation2ds.add(commandSwerveDrivetrain.getPose().getTranslation());
    //     return TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(robotTranslation2ds.get(0), new Rotation2d(0)),
    //         robotTranslation2ds,
    //         commandSwerveDrivetrain.getPose(),
    //         new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
    // }
}
