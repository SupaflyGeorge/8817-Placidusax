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

public class PhotonVisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public PhotonVisionIO() {
    camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

    poseEstimator =
        new PhotonPoseEstimator(
            Constants.VisionConstants.FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.ROBOT_TO_CAM);

    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<VisionUpdate> getVisionUpdate(Pose2d currentEstimatedPose) {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return Optional.empty();

    PhotonTrackedTarget best = result.getBestTarget();

    // Ambiguity filter
    double ambiguity = best.getPoseAmbiguity();
    if (ambiguity >= 0.0 && ambiguity > Constants.VisionConstants.MAX_AMBIGUITY) {
      return Optional.empty();
    }

    // Rough distance filter (camera->target)
    double dist = best.getBestCameraToTarget().getTranslation().getNorm();
    if (dist > Constants.VisionConstants.MAX_DISTANCE_METERS) {
      return Optional.empty();
    }

    // IMPORTANT: helps solve ambiguity / stability
    poseEstimator.setReferencePose(currentEstimatedPose);

    Optional<EstimatedRobotPose> estOpt = poseEstimator.update(result);
    if (estOpt.isEmpty()) return Optional.empty();

    EstimatedRobotPose est = estOpt.get();

    int numTags = result.targets.size();
    Matrix<N3, N1> stdDevs =
        (numTags > 1) ? Constants.VisionConstants.MULTI_TAG_STD_DEVS
                      : Constants.VisionConstants.SINGLE_TAG_STD_DEVS;

    return Optional.of(new VisionUpdate(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs));
  }

  public static record VisionUpdate(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
}