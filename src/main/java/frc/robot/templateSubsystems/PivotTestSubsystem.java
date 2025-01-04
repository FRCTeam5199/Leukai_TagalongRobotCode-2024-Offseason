package frc.robot.templateSubsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

public class PivotTestSubsystem extends TemplateSubsystem {
    public PivotTestSubsystem() {
        super(Type.PIVOT, 24,
                new TrapezoidProfile.Constraints(1, 2),
                new PID(50, 0, 0), new FeedForward(.175, .707, 2.5),
                .75, .75, new double[][]{{42.4286, 1}});
        configureMotor(true, true, 80, 80);
        configurePivot(0, 57, 13.5);
        configureEncoder(30, "rio", .930176, 1, 42.4286);
    }
}
