package frc.robot.templateSubsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

public class RollerTestSubsystem extends AbstractSubsystem {
    public RollerTestSubsystem() {
        super(Type.ROLLER, new int[]{25},
                new TrapezoidProfile.Constraints(90, 200),
                new PID(0, 0, 0), new FeedForward(0.23, 0.11904761904761904761904761904762),
                3, 3, new double[][]{{1, 1}});
        configureMotor(true, false, new double[][]{{80, 80}});
    }
}
