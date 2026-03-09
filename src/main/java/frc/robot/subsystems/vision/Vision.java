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
  private final GenericEntry sbTagId     = tab.add("Best Tag ID", -1).getEntry();
  private final GenericEntry sbYawDeg    = tab.add("Best Yaw (deg)", 0.0).getEntry();
  private final GenericEntry sbDistX     = tab.add("Cam->Tag X (m)", 0.0).getEntry();
  private final GenericEntry sbDistY     = tab.add("Cam->Tag Y (m)", 0.0).getEntry();

  // Your center-yaw debug
  private final GenericEntry sbCenterYawDeg = tab.add("CenterYaw (deg)", 0.0).getEntry();
  private final GenericEntry sbCenterTagsUsed = tab.add("CenterYaw Tags Used", 0.0).getEntry();

  // Tuning knobs
  private final GenericEntry sbAimKp     = tab.add("Aim kP", 4.0).getEntry();
  private final GenericEntry sbAimKd     = tab.add("Aim kD", 0.15).getEntry();
  private final GenericEntry sbAimTolDeg = tab.add("Aim Tol (deg)", 2.0).getEntry();

  private final GenericEntry sbStrafeKp   = tab.add("Strafe kP", 2.0).getEntry();
  private final GenericEntry sbStrafeTolM = tab.add("Strafe Tol (m)", 0.05).getEntry();

  public Vision() {
    yawPid.enableContinuousInput(-Math.PI, Math.PI);
    yawPid.setTolerance(Units.degreesToRadians(2.0));
    strafePid.setTolerance(0.05);
  }

  /** Feed multi-tag pose into CTRE estimator. Call this every loop. */
  public void updateVisionPose(CommandSwerveDrivetrain drivetrain) {
    Pose2d curPose = drivetrain.getState().Pose;

    io.getVisionUpdate(curPose).ifPresent(update -> {
      drivetrain.addVisionMeasurement(update.pose(), update.timestampSeconds(), update.stdDevs());
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

  /** Returns averaged yaw (rad) based on your hub-centers logic + how many tags were used. */
  private CenterYawResult calcHubCenterYawRadWithCount() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return new CenterYawResult(0.0, 0);

    List<Integer> hubCenters = new ArrayList<>();
    List<Integer> hubIDs = new ArrayList<>();

    boolean blueAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    if (blueAlliance) {
      hubCenters.addAll(List.of(18, 24, 26));
      hubIDs.addAll(List.of(21, 24, 19, 20, 18, 27));
    } else {
      hubCenters.addAll(List.of(2, 8, 10));
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
      if ((t1.getFiducialId() - t0.getFiducialId()) == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)){
        for (var t : targets) {
          yawTotal += Units.degreesToRadians(t.getYaw());
          tagCount++;
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

  

  /** Calculate omega (rad/s) to aim at the hub center yaw. Falls back to bestTarget yaw if needed. */
  public double calcAimOmegaRadPerSec() {
    CenterYawResult center = calcHubCenterYawRadWithCount();

    // Prefer your center yaw if we actually used tags
    if (center.tagsUsed > 0) {
      return yawPid.calculate(center.yawRad, 0.0);
    }

    // Fallback to best target yaw if any target exists
    return getBestTarget()
        .map(t -> yawPid.calculate(-Units.degreesToRadians(t.getYaw()), 0.0))
        .orElse(0.0);
  }

  /** Robot-relative “auto strafe” correction (vy m/s). */
  public double calcStrafeVyRobotMps() {
    return getBestTarget().map(t -> {
      double lateral = t.getBestCameraToTarget().getTranslation().getY(); // +left
      double vy = strafePid.calculate(lateral, 0.0);

      // deadzone finish
      if (Math.abs(lateral) < strafePid.getErrorTolerance()) vy = 0.0;

      // clamp authority
      return Math.max(-1.5, Math.min(1.5, vy));
    }).orElse(0.0);
  }

  /** Robot-relative “auto forward” correction (vx m/s). Keeps your old negative behavior. */
  public double calcApproachVxRobotMps(double stopDistanceMeters, double approachSpeedMps) {
    return getBestTarget().map(t -> {
      double x = t.getBestCameraToTarget().getTranslation().getX(); // +forward in camera frame
      double dist = Math.abs(x);
      if (dist > stopDistanceMeters) {
        return +approachSpeedMps;
      }
      return 0.0;
    }).orElse(0.0);
  }

  /** Refresh Shuffleboard + allow live tuning. */
  @Override
  public void periodic() {
    // Live tuning
    yawPid.setP(sbAimKp.getDouble(4.0));
    yawPid.setD(sbAimKd.getDouble(0.15));
    yawPid.setTolerance(Units.degreesToRadians(sbAimTolDeg.getDouble(2.0)));

    strafePid.setP(sbStrafeKp.getDouble(2.0));
    strafePid.setTolerance(sbStrafeTolM.getDouble(0.05));

    // Best target debug
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

    // Your center-yaw debug
    CenterYawResult center = calcHubCenterYawRadWithCount();
    sbCenterYawDeg.setDouble(Units.radiansToDegrees(center.yawRad));
    sbCenterTagsUsed.setDouble(center.tagsUsed);
  }
}