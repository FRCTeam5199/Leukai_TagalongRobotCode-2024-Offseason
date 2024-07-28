package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class ApriltagSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private double lastEstTimestamp = 0;

    public ApriltagSubsystem() {
        camera = new PhotonCamera(Constants.Vision.kCameraName);

        photonEstimator =
                new PhotonPoseEstimator(
                        Constants.Vision.kTagLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, Constants.Vision.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
//        System.out.println("Distance from Red Speaker: " + getDistanceFromRedSpeaker());
    }

    public double getDistanceFromRedSpeaker() {
        return Math.sqrt(Math.pow((16.579342 - drivetrain.getPose().getX()), 2)
                + Math.pow((5.547867999 - drivetrain.getPose().getY()), 2));
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        photonEstimator.setReferencePose(drivetrain.getPose());

        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            return photonEstimator.update();
        } else {
            return Optional.empty();
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
