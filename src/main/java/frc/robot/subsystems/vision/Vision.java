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

public class Vision extends SubsystemBase {
  private final PhotonVisionIO io = new PhotonVisionIO();

  // For tag assist (aim/strafe)
  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

  // Aim PID: yaw rad -> omega rad/s
  private final PIDController yawPid = new PIDController(4.0, 0.0, 0.15);
  // Strafe PID: camToTarget Y meters -> vy robot m/s
  private final PIDController strafePid = new PIDController(2.0, 0.0, 0.0);

  // Shuffleboard
  private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  private final GenericEntry sbHasTarget = tab.add("Has Target", false).getEntry();
  private final GenericEntry sbTagId = tab.add("Best Tag ID", -1).getEntry();
  private final GenericEntry sbYawDeg = tab.add("Best Yaw (deg)", 0.0).getEntry();
  private final GenericEntry sbDistX = tab.add("Cam->Tag X (m)", 0.0).getEntry();
  private final GenericEntry sbDistY = tab.add("Cam->Tag Y (m)", 0.0).getEntry();

  // Center-yaw debug
  private final GenericEntry sbCenterYawDeg = tab.add("CenterYaw (deg)", 0.0).getEntry();
  private final GenericEntry sbCenterTagsUsed = tab.add("CenterYaw Tags Used", 0.0).getEntry();

  // Tuning knobs
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

  // Safety limits for accepting vision pose updates
  private static final double MAX_VISION_TRANSLATION_ERROR_M = 1.0;
  private static final double MAX_VISION_ROTATION_ERROR_DEG = 20.0;
  private static final double MAX_VISION_LINEAR_SPEED_MPS = 1.5;
  private static final double MAX_VISION_OMEGA_DEGPS = 180.0;

  public Vision() {
    yawPid.enableContinuousInput(-Math.PI, Math.PI);
    yawPid.setTolerance(Units.degreesToRadians(2.0));
    strafePid.setTolerance(0.05);
  }

  /**
   * Feed vision pose into CTRE estimator, but only when the measurement looks safe.
   * This keeps aiming/tracking working without letting pose jump all over the field.
   */
  public void updateVisionPose(CommandSwerveDrivetrain drivetrain) {
    Pose2d curPose = drivetrain.getState().Pose;

    io.getVisionUpdate(curPose).ifPresent(update -> {
      Pose2d visionPose = update.pose();

      double dx = visionPose.getX() - curPose.getX();
      double dy = visionPose.getY() - curPose.getY();
      double translationError = Math.hypot(dx, dy);

      double rotationErrorDeg =
          Math.abs(visionPose.getRotation().minus(curPose.getRotation()).getDegrees());

      double robotSpeed =
          Math.hypot(
              drivetrain.getState().Speeds.vxMetersPerSecond,
              drivetrain.getState().Speeds.vyMetersPerSecond);

      double omegaDegPerSec =
          Math.abs(Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));

      boolean accept =
          translationError <= MAX_VISION_TRANSLATION_ERROR_M
              && rotationErrorDeg <= MAX_VISION_ROTATION_ERROR_DEG
              && robotSpeed <= MAX_VISION_LINEAR_SPEED_MPS
              && omegaDegPerSec <= MAX_VISION_OMEGA_DEGPS;

      sbVisionPoseErrorM.setDouble(translationError);
      sbVisionRotErrorDeg.setDouble(rotationErrorDeg);
      sbVisionRobotSpeed.setDouble(robotSpeed);
      sbVisionOmegaDegPerSec.setDouble(omegaDegPerSec);
      sbVisionAccepted.setBoolean(accept);

      if (accept) {
        drivetrain.addVisionMeasurement(
            update.pose(),
            update.timestampSeconds(),
            update.stdDevs());
      }
    });
  }

  /** Latest target (best target) if present. */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return Optional.empty();
    return Optional.of(res.getBestTarget());
  }

  /** True if the camera currently sees any targets. */
  public boolean hasTarget() {
    PhotonPipelineResult res = camera.getLatestResult();
    return res != null && res.hasTargets();
  }

  private static class CenterYawResult {
    final double yawRad;
    final int tagsUsed;

    CenterYawResult(double yawRad, int tagsUsed) {
      this.yawRad = yawRad;
      this.tagsUsed = tagsUsed;
    }
  }

  /** Returns averaged yaw (rad) based on hub-centers logic + how many tags were used. */
  private CenterYawResult calcHubCenterYawRadWithCount() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return new CenterYawResult(0.0, 0);

    List<Integer> hubCenters = new ArrayList<>();
    List<Integer> hubIDs = new ArrayList<>();

    boolean blueAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;

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

    if (targets.size() == 1) {


      
      var t = targets.get(0);
      if (hubIDs.contains(t.getFiducialId())) {
        yawTotal += Units.degreesToRadians(t.getYaw());
        tagCount++;
      }
    } else if (targets.size() == 2) {
      var t0 = targets.get(0);
      var t1 = targets.get(1);

      if ((t1.getFiducialId() - t0.getFiducialId())
          == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)
          || !(hubCenters.contains(t0.getFiducialId()) || hubCenters.contains(t1.getFiducialId())) ) {
        for (var t : targets) {
          if (hubIDs.contains(t.getFiducialId())){
            yawTotal += Units.degreesToRadians(t.getYaw());
            tagCount++;
          }
        }
      } else {
        for (var t : targets) {
          if (hubCenters.contains(t.getFiducialId())) {
            yawTotal += Units.degreesToRadians(t.getYaw());
            tagCount++;
          }
        }
      }
    } else if (targets.size() == 3) {
      for (var t : targets) {
        if (hubCenters.contains(t.getFiducialId())) {
          yawTotal += Units.degreesToRadians(t.getYaw());
          tagCount++;
        }
      }
    } else {
      for (var t : targets) {
        if (hubIDs.contains(t.getFiducialId())) {
          yawTotal += Units.degreesToRadians(t.getYaw());
          tagCount++;
        }
      }
    }

    if (tagCount == 0) return new CenterYawResult(0.0, 0);
    return new CenterYawResult(yawTotal / tagCount, tagCount);
  }

  /** Calculate omega (rad/s) to aim at the hub center yaw. Falls back to best target yaw if needed. */
  public double calcAimOmegaRadPerSec() {
    CenterYawResult center = calcHubCenterYawRadWithCount();

    if (center.tagsUsed > 0) {
      return yawPid.calculate(center.yawRad, 0.0);
    }

    return getBestTarget()
        .map(t -> yawPid.calculate(-Units.degreesToRadians(t.getYaw()), 0.0))
        .orElse(0.0);
  }

  /** Robot-relative auto strafe correction (vy m/s). */
  public double calcStrafeVyRobotMps() {
    return getBestTarget()
        .map(
            t -> {
              double lateral = t.getBestCameraToTarget().getTranslation().getY(); // +left
              double vy = strafePid.calculate(lateral, 0.0);

              if (Math.abs(lateral) < strafePid.getErrorTolerance()) vy = 0.0;

              return Math.max(-1.5, Math.min(1.5, vy));
            })
        .orElse(0.0);
  }

  /** Robot-relative auto forward correction (vx m/s). */
  public double calcApproachVxRobotMps(double stopDistanceMeters, double approachSpeedMps) {
    return getBestTarget()
        .map(
            t -> {
              double x = t.getBestCameraToTarget().getTranslation().getX(); // +forward in camera frame
              double dist = Math.abs(x);
              if (dist > stopDistanceMeters) {
                return +approachSpeedMps;
              }
              return 0.0;
            })
        .orElse(0.0);
  }

  @Override
  public void periodic() {
    yawPid.setP(sbAimKp.getDouble(4.0));
    yawPid.setD(sbAimKd.getDouble(0.15));
    yawPid.setTolerance(Units.degreesToRadians(sbAimTolDeg.getDouble(2.0)));

    strafePid.setP(sbStrafeKp.getDouble(2.0));
    strafePid.setTolerance(sbStrafeTolM.getDouble(0.05));

    var opt = getBestTarget();
    sbHasTarget.setBoolean(opt.isPresent());

    if (opt.isPresent()) {
      var t = opt.get();
      sbTagId.setDouble(t.getFiducialId());
      sbYawDeg.setDouble(t.getYaw());
      sbDistX.setDouble(t.getBestCameraToTarget().getTranslation().getX());
      sbDistY.setDouble(t.getBestCameraToTarget().getTranslation().getY());
    } else {
      sbTagId.setDouble(-1);
      sbYawDeg.setDouble(0.0);
      sbDistX.setDouble(0.0);
      sbDistY.setDouble(0.0);
    }

    CenterYawResult center = calcHubCenterYawRadWithCount();
    sbCenterYawDeg.setDouble(Units.radiansToDegrees(center.yawRad));
    sbCenterTagsUsed.setDouble(center.tagsUsed);
  }
}