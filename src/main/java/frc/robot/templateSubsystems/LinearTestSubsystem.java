package frc.robot.templateSubsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

public class LinearTestSubsystem extends TemplateSubsystem {
    public LinearTestSubsystem() {
        super(Type.LINEAR, 22,
                new TrapezoidProfile.Constraints(3, 7.5),
                new PID(2, 0, 0), new FeedForward(.4, .205, 8.3333333333333333333333333333333),
                .015, .015, new double[][]{{48, 14}, {48, 24}});
        configureMotor(false, true, 80, 80);
        configureFollowerMotor(23);
        configureLinearMech(1.273, 0, 100);
    }
}
