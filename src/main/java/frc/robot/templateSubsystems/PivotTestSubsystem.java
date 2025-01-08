package frc.robot.templateSubsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

public class PivotTestSubsystem extends AbstractSubsystem {
    public PivotTestSubsystem() {
        super(Type.PIVOT, new int[]{24},
                new TrapezoidProfile.Constraints(1, 2),
                new PID(50, 0, 0), new FeedForward(0.175, 0.707, 2.5),
                0.75, 0.75, new double[][]{{42.4286, 1}});
        configureMotor(true, true, new double[][]{{80, 80}});
        configurePivot(0, 57, 13.5);
        configureEncoder(30, "rio", 0.930176, 1, 42.4286);
    }
}
