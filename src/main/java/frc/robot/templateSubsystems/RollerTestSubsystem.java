package frc.robot.templateSubsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

public class RollerTestSubsystem extends TemplateSubsystem {
    public RollerTestSubsystem() {
        super(Type.ROLLER, 25,
                new TrapezoidProfile.Constraints(90, 200),
                new PID(0, 0, 0), new FeedForward(.23, .11904761904761904761904761904762),
                3, 3, new double[][]{{1, 1}});
        configureMotor(true, false, 80, 80);
    }
}
