package frc.robot.subsystems;

import frc.robot.templateSubsystems.RollerSubsystemTemplate;

import static frc.robot.constants.Constants.*;

public class TestRollerSubsystem extends RollerSubsystemTemplate {

    private static TestRollerSubsystem testRollerSubsystem;

    private TestRollerSubsystem() {
        super(TestRollerConstants.INDEXER_ID, TestRollerConstants.INDEXER_CONSTRAINTS, TestRollerConstants.INDEXER_PID,
                TestRollerConstants.INDEXER_FEEDFORWARD, TestRollerConstants.INDEXER_LOWER_TOLERANCE,
                TestRollerConstants.INDEXER_UPPER_TOLERANCE, TestRollerConstants.INDEXER_GEAR_RATIO);

        configureMotor(false, true, 40, 40);
    }

    public static TestRollerSubsystem getInstance() {
        if (testRollerSubsystem == null) testRollerSubsystem = new TestRollerSubsystem();
        return testRollerSubsystem;
    }
}
