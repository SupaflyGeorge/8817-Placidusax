package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Vision subsystem: handles two jobs:
 *
 *   1. POSE ESTIMATION — feeds filtered AprilTag measurements into the
 *      drivetrain's Kalman filter so odometry stays accurate.
 *      Safety gates reject updates that look wrong (too far from current
 *      pose, robot moving too fast, etc).
 *
 *   2. TAG AIMING — calculates rotation speed (omega) to point the robot
 *      at the hub's center. Uses a PID controller on the average yaw of
 *      visible hub tags. The ShootOnMoveCommand uses this for auto-aim.
 *
 * All tuning knobs (PID gains, tolerances) are on Shuffleboard for live adjustment.
 */
public class Vision extends SubsystemBase {
  private final PhotonVisionIO io = new PhotonVisionIO();
  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

  // Yaw PID: error in radians -> output in rad/s rotation command
  private final PIDController yawPid = new PIDController(4.0, 0.0, 0.15);
  // Strafe PID: lateral offset in meters -> strafe speed (currently unused in teleop)
  private final PIDController strafePid = new PIDController(2.0, 0.0, 0.0);

  // --- Shuffleboard entries for live debugging ---
  private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  private final GenericEntry sbHasTarget = tab.add("Has Target", false).getEntry();
  private final GenericEntry sbTagId = tab.add("Best Tag ID", -1).getEntry();
  private final GenericEntry sbYawDeg = tab.add("Best Yaw (deg)", 0.0).getEntry();
  private final GenericEntry sbDistX = tab.add("Cam->Tag X (m)", 0.0).getEntry();
  private final GenericEntry sbDistY = tab.add("Cam->Tag Y (m)", 0.0).getEntry();
  private final GenericEntry sbCenterYawDeg = tab.add("CenterYaw (deg)", 0.0).getEntry();
  private final GenericEntry sbCenterTagsUsed = tab.add("CenterYaw Tags Used", 0.0).getEntry();

  // Tunable PID gains on Shuffleboard
  private final GenericEntry sbAimKp = tab.add("Aim kP", 4.0).getEntry();
  private final GenericEntry sbAimKd = tab.add("Aim kD", 0.15).getEntry();
  private final GenericEntry sbAimTolDeg = tab.add("Aim Tol (deg)", 2.0).getEntry();
  private final GenericEntry sbStrafeKp = tab.add("Strafe kP", 2.0).getEntry();
  private final GenericEntry sbStrafeTolM = tab.add("Strafe Tol (m)", 0.05).getEntry();

  // Vision pose gating debug
  private final GenericEntry sbVisionAccepted = tab.add("Pose Accepted", false).getEntry();
  private final GenericEntry sbVisionPoseErrorM = tab.add("Pose Error (m)", 0.0).getEntry();
  private final GenericEntry sbVisionRotErrorDeg = tab.add("Pose Rot Error (deg)", 0.0).getEntry();
  private final GenericEntry sbVisionRobotSpeed = tab.add("Robot Speed (mps)", 0.0).getEntry();
  private final GenericEntry sbVisionOmegaDegPerSec = tab.add("Robot Omega (degps)", 0.0).getEntry();

  // Safety thresholds: reject vision if any of these are exceeded
  private static final double MAX_VISION_TRANSLATION_ERROR_M = 1.0;
  private static final double MAX_VISION_ROTATION_ERROR_DEG = 20.0;
  private static final double MAX_VISION_LINEAR_SPEED_MPS = 1.5;
  private static final double MAX_VISION_OMEGA_DEGPS = 180.0;

  public Vision() {
    yawPid.enableContinuousInput(-Math.PI, Math.PI); // wrap-around for heading
    yawPid.setTolerance(Units.degreesToRadians(2.0));
    strafePid.setTolerance(0.05);
  }

  /**
   * Feed vision pose into the drivetrain estimator, but only when it looks safe.
   * We check: is the pose close to where we think we are? Is the robot moving slowly enough?
   * This prevents wild pose jumps from bad detections.
   */
  public void updateVisionPose(CommandSwerveDrivetrain drivetrain) {
    Pose2d curPose = drivetrain.getState().Pose;
    io.getVisionUpdate(curPose).ifPresent(update -> {
      Pose2d visionPose = update.pose();
      double translationError = Math.hypot(visionPose.getX() - curPose.getX(), visionPose.getY() - curPose.getY());
      double rotationErrorDeg = Math.abs(visionPose.getRotation().minus(curPose.getRotation()).getDegrees());
      double robotSpeed = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
      double omegaDegPerSec = Math.abs(Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));

      boolean accept = translationError <= MAX_VISION_TRANSLATION_ERROR_M
          && rotationErrorDeg <= MAX_VISION_ROTATION_ERROR_DEG
          && robotSpeed <= MAX_VISION_LINEAR_SPEED_MPS
          && omegaDegPerSec <= MAX_VISION_OMEGA_DEGPS;

      sbVisionPoseErrorM.setDouble(translationError);
      sbVisionRotErrorDeg.setDouble(rotationErrorDeg);
      sbVisionRobotSpeed.setDouble(robotSpeed);
      sbVisionOmegaDegPerSec.setDouble(omegaDegPerSec);
      sbVisionAccepted.setBoolean(accept);

      if (accept) drivetrain.addVisionMeasurement(update.pose(), update.timestampSeconds(), update.stdDevs());
    });
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return Optional.empty();
    return Optional.of(res.getBestTarget());
  }

  public boolean hasTarget() {
    PhotonPipelineResult res = camera.getLatestResult();
    return res != null && res.hasTargets();
  }

  // --- Hub center yaw calculation ---
  // The hub has multiple tags. Depending on how many we see, we pick
  // the best strategy: center tags only, all tags averaged, etc.

  private static class CenterYawResult {
    final double yawRad;
    final int tagsUsed;
    CenterYawResult(double yawRad, int tagsUsed) { this.yawRad = yawRad; this.tagsUsed = tagsUsed; }
  }

  private CenterYawResult calcHubCenterYawRadWithCount() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return new CenterYawResult(0.0, 0);

    List<Integer> hubCenters = new ArrayList<>();
    List<Integer> hubIDs = new ArrayList<>();
    boolean blueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    if (blueAlliance) {
      hubCenters.addAll(List.of(18, 21, 26));
      hubIDs.addAll(List.of(21, 24, 25, 26, 18, 27));
    } else {
      hubCenters.addAll(List.of(2, 5, 10));
      hubIDs.addAll(List.of(2, 11, 8, 5, 9, 10));
    }

    double yawTotal = 0.0;
    int tagCount = 0;
    var targets = res.getTargets();

    // Strategy depends on how many hub tags we see
    if (targets.size() == 1) {
      var t = targets.get(0);
      if (hubIDs.contains(t.getFiducialId())) { yawTotal += Units.degreesToRadians(t.getYaw()); tagCount++; }
    } else if (targets.size() == 2) {
      var t0 = targets.get(0); var t1 = targets.get(1);
      if ((t1.getFiducialId() - t0.getFiducialId()) == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)
          || !(hubCenters.contains(t0.getFiducialId()) || hubCenters.contains(t1.getFiducialId()))) {
        for (var t : targets) { if (hubIDs.contains(t.getFiducialId())) { yawTotal += Units.degreesToRadians(t.getYaw()); tagCount++; } }
      } else {
        for (var t : targets) { if (hubCenters.contains(t.getFiducialId())) { yawTotal += Units.degreesToRadians(t.getYaw()); tagCount++; } }
      }
    } else if (targets.size() == 3) {
      for (var t : targets) { if (hubCenters.contains(t.getFiducialId())) { yawTotal += Units.degreesToRadians(t.getYaw()); tagCount++; } }
    } else {
      for (var t : targets) { if (hubIDs.contains(t.getFiducialId())) { yawTotal += Units.degreesToRadians(t.getYaw()); tagCount++; } }
    }

    if (tagCount == 0) return new CenterYawResult(0.0, 0);
    return new CenterYawResult(yawTotal / tagCount, tagCount);
  }

  /**
   * Calculate rotation speed (rad/s) to aim at the hub center.
   * Uses averaged hub tag yaw, falling back to best-target yaw if no hub tags found.
   */
  public double calcAimOmegaRadPerSec() {
    CenterYawResult center = calcHubCenterYawRadWithCount();
    if (center.tagsUsed > 0) return yawPid.calculate(center.yawRad, 0.0);
    return getBestTarget().map(t -> yawPid.calculate(-Units.degreesToRadians(t.getYaw()), 0.0)).orElse(0.0);
  }

  /** Strafe correction to center on the target laterally. */
  public double calcStrafeVyRobotMps() {
    return getBestTarget().map(t -> {
      double lateral = t.getBestCameraToTarget().getTranslation().getY();
      double vy = strafePid.calculate(lateral, 0.0);
      if (Math.abs(lateral) < strafePid.getErrorTolerance()) vy = 0.0;
      return Math.max(-1.5, Math.min(1.5, vy));
    }).orElse(0.0);
  }

  /** Forward correction to approach a target to a stop distance. */
  public double calcApproachVxRobotMps(double stopDistanceMeters, double approachSpeedMps) {
    return getBestTarget().map(t -> {
      double x = t.getBestCameraToTarget().getTranslation().getX();
      return Math.abs(x) > stopDistanceMeters ? +approachSpeedMps : 0.0;
    }).orElse(0.0);
  }

  @Override
  public void periodic() {
    // Update PID from Shuffleboard so we can tune live
    yawPid.setP(sbAimKp.getDouble(4.0));
    yawPid.setD(sbAimKd.getDouble(0.15));
    yawPid.setTolerance(Units.degreesToRadians(sbAimTolDeg.getDouble(2.0)));
    strafePid.setP(sbStrafeKp.getDouble(2.0));
    strafePid.setTolerance(sbStrafeTolM.getDouble(0.05));

    // Publish best target info
    var opt = getBestTarget();
    sbHasTarget.setBoolean(opt.isPresent());
    if (opt.isPresent()) {
      var t = opt.get();
      sbTagId.setDouble(t.getFiducialId());
      sbYawDeg.setDouble(t.getYaw());
      sbDistX.setDouble(t.getBestCameraToTarget().getTranslation().getX());
      sbDistY.setDouble(t.getBestCameraToTarget().getTranslation().getY());
    } else { sbTagId.setDouble(-1); sbYawDeg.setDouble(0.0); sbDistX.setDouble(0.0); sbDistY.setDouble(0.0); }

    CenterYawResult center = calcHubCenterYawRadWithCount();
    sbCenterYawDeg.setDouble(Units.radiansToDegrees(center.yawRad));
    sbCenterTagsUsed.setDouble(center.tagsUsed);
  }
}
