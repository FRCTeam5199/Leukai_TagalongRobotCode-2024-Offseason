package frc.robot.templateSubsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

public class FlywheelTestSubsystem extends AbstractSubsystem {
    public FlywheelTestSubsystem() {
        super(Type.FLYWHEEL, new int[]{26},
                new TrapezoidProfile.Constraints(90, 500),
                new PID(0, 0, 0), new FeedForward(.08, 0.16393442622950819672131147540984),
                5, 5, new double[][]{{1, 1}});
        configureMotor(false, true, new double[][]{{80, 80}});
    }
}
