package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Wraps PhotonVision's pose estimator to produce filtered pose measurements.
 *
 * Flow:
 *   1. Get the latest camera frame
 *   2. Reject if the best target is too ambiguous or too far away
 *   3. Feed the reference pose (current odometry) to help resolve ambiguity
 *   4. Run the multi-tag PNP solver (falls back to single-tag if needed)
 *   5. Return the estimated pose + timestamp + appropriate std devs
 */
public class PhotonVisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public PhotonVisionIO() {
    camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
    poseEstimator = new PhotonPoseEstimator(
        Constants.VisionConstants.FIELD_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.VisionConstants.ROBOT_TO_CAM);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<VisionUpdate> getVisionUpdate(Pose2d currentEstimatedPose) {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return Optional.empty();

    PhotonTrackedTarget best = result.getBestTarget();

    // Reject high-ambiguity detections (noisy / unreliable)
    double ambiguity = best.getPoseAmbiguity();
    if (ambiguity >= 0.0 && ambiguity > Constants.VisionConstants.MAX_AMBIGUITY) return Optional.empty();

    // Reject tags that are too far away (accuracy drops off)
    double dist = best.getBestCameraToTarget().getTranslation().getNorm();
    if (dist > Constants.VisionConstants.MAX_DISTANCE_METERS) return Optional.empty();

    // Feed current pose to help the solver pick the right solution
    poseEstimator.setReferencePose(currentEstimatedPose);

    Optional<EstimatedRobotPose> estOpt = poseEstimator.update(result);
    if (estOpt.isEmpty()) return Optional.empty();

    EstimatedRobotPose est = estOpt.get();

    // Multi-tag is more trustworthy than single-tag
    int numTags = result.targets.size();
    Matrix<N3, N1> stdDevs = (numTags > 1)
        ? Constants.VisionConstants.MULTI_TAG_STD_DEVS
        : Constants.VisionConstants.SINGLE_TAG_STD_DEVS;

    return Optional.of(new VisionUpdate(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs));
  }

  /** Immutable data class holding one vision measurement. */
  public static record VisionUpdate(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
}