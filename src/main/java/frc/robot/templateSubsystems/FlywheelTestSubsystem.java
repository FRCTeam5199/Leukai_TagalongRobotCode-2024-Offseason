package frc.robot.templateSubsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

public class FlywheelTestSubsystem extends TemplateSubsystem {
    public FlywheelTestSubsystem() {
        super(Type.FLYWHEEL, 26,
                new TrapezoidProfile.Constraints(90, 500),
                new PID(0, 0, 0), new FeedForward(.08, .16393442622950819672131147540984),
                5, 5, new double[][]{{1, 1}});
        configureMotor(true, false, 80, 80);
    }
}
