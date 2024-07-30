package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.enums.FieldVersions;

import javax.swing.text.Utilities;

public class Constants {
    public static final RobotVersions CRobot = RobotVersions.COMP_BOT;
    public static final FieldVersions CField = FieldVersions.THEORETICAL_FIELD;

    // These are high level robot configurations that fundamentally change robot
    // behavior and control schemes, this also allows for manual overrides. I don't like this,
    // would love to see alternatives... These should really become part of the top level
    // RobotVersions

    // TODO move
    /* ========== ROBOT OFFSET DIMENSIONS ========== */
    public static final double LOOP_PERIOD_MS = 20.0;
    public static final double LOOP_PERIOD_S = Units.millisecondsToSeconds(LOOP_PERIOD_MS);

    /* ========== ROBOT SPEEDS ========== */
    public static final double NOTE_TRAVEL_INSIDE_DELTA_S = 1.0; // TODO: calculate

    /* ========== NATURAL CONSTANTS ========== */
    public static final double GRAVITY_MPS2 = -9.81;
    /* For Drivetrain Auto */
    public static final double TRANSLATION_MAX_TRIM_SPEED_MPS = 1;
    public static final Rotation2d ANGLE_MAX_TRIM_SPEED_DPS = Rotation2d.fromDegrees(90.0);

    public static final class CTREConstants {
        /* ============= Falcon Constants2 ============= */
        // ticks per motor rotation
        // TODO: measure this free speed on blocks
        // 6380 +/- 10%
        // TODO FOC?
        public static final double MAX_KRAKEN_FOC_RPM = 5800.0;
    }

    public static final class Sensors {
        public static final boolean INVERT_GYRO = false;

        public static final Rotation2d GYRO_ZERO_BLUE = Rotation2d.fromDegrees(0);
        public static final Rotation2d GYRO_ZERO_RED = Rotation2d.fromDegrees(180);

        public static final double POV_ZERO_BLUE_DEG = 0;
        public static final double POV_ZERO_RED_DEG = POV_ZERO_BLUE_DEG + 180;
    }

    public static final class Control {
        public static final double STICK_DEADBAND = 0.04; // TODO: tune
        public static final double STICK_NET_DEADBAND = 0.06; // TODO: tune
        public static final boolean IS_OPEN_LOOP = false; // swerve
    }

    public static class Vision {
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.07, 0.07, 0);
        public static final String kCameraName = "Camera";

        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(-0.1006348, 0.0, 0.4481322), new Rotation3d(0, -.29, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        public static final double[] RED_SPEAKER_COORDINATES = new double[]{16.54, 5.547};
        public static final double[] BLUE_SPEAKER_COORDINATES = new double[]{0, 5.547};
    }
}
