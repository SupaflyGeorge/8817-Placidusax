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
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
  private final PhotonVisionIO io = new PhotonVisionIO();
  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

  private final PIDController yawPid = new PIDController(4.0, 0.0, 0.15);
  private final PIDController strafePid = new PIDController(2.0, 0.0, 0.0);

  private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  private final GenericEntry sbHasTarget = tab.add("Has Target", false).getEntry();
  private final GenericEntry sbTagId = tab.add("Best Tag ID", -1).getEntry();
  private final GenericEntry sbYawDeg = tab.add("Best Yaw (deg)", 0.0).getEntry();
  private final GenericEntry sbDistX = tab.add("Cam->Tag X (m)", 0.0).getEntry();
  private final GenericEntry sbDistY = tab.add("Cam->Tag Y (m)", 0.0).getEntry();
  private final GenericEntry sbRobotDistX = tab.add("Robot->Tag X (m)", 0.0).getEntry();
  private final GenericEntry sbRobotDistY = tab.add("Robot->Tag Y (m)", 0.0).getEntry();
  private final GenericEntry sbCenterYawDeg = tab.add("CenterYaw (deg)", 0.0).getEntry();
  private final GenericEntry sbCenterTagsUsed = tab.add("CenterYaw Tags Used", 0.0).getEntry();

  private final GenericEntry sbAimKp = tab.add("Aim kP", 4.0).getEntry();
  private final GenericEntry sbAimKd = tab.add("Aim kD", 0.15).getEntry();
  private final GenericEntry sbAimTolDeg = tab.add("Aim Tol (deg)", 2.0).getEntry();
  private final GenericEntry sbStrafeKp = tab.add("Strafe kP", 2.0).getEntry();
  private final GenericEntry sbStrafeTolM = tab.add("Strafe Tol (m)", 0.05).getEntry();

  private final GenericEntry sbVisionAccepted = tab.add("Pose Accepted", false).getEntry();
  private final GenericEntry sbVisionPoseErrorM = tab.add("Pose Error (m)", 0.0).getEntry();
  private final GenericEntry sbVisionRotErrorDeg = tab.add("Pose Rot Error (deg)", 0.0).getEntry();
  private final GenericEntry sbVisionRobotSpeed = tab.add("Robot Speed (mps)", 0.0).getEntry();
  private final GenericEntry sbVisionOmegaDegPerSec = tab.add("Robot Omega (degps)", 0.0).getEntry();

  private static final double MAX_VISION_TRANSLATION_ERROR_M = 1.0;
  private static final double MAX_VISION_ROTATION_ERROR_DEG = 20.0;
  private static final double MAX_VISION_LINEAR_SPEED_MPS = 1.5;
  private static final double MAX_VISION_OMEGA_DEGPS = 180.0;

  public Vision() {
    yawPid.enableContinuousInput(-Math.PI, Math.PI);
    yawPid.setTolerance(Units.degreesToRadians(2.0));
    strafePid.setTolerance(0.05);
  }

  public void updateVisionPose(CommandSwerveDrivetrain drivetrain) {
    Pose2d curPose = drivetrain.getState().Pose;
    io.getVisionUpdate(curPose).ifPresent(update -> {
      Pose2d visionPose = update.pose();
      double translationError = Math.hypot(
          visionPose.getX() - curPose.getX(),
          visionPose.getY() - curPose.getY());
      double rotationErrorDeg = Math.abs(
          visionPose.getRotation().minus(curPose.getRotation()).getDegrees());
      double robotSpeed = Math.hypot(
          drivetrain.getState().Speeds.vxMetersPerSecond,
          drivetrain.getState().Speeds.vyMetersPerSecond);
      double omegaDegPerSec = Math.abs(
          Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));

      boolean accept = translationError <= MAX_VISION_TRANSLATION_ERROR_M
          && rotationErrorDeg <= MAX_VISION_ROTATION_ERROR_DEG
          && robotSpeed <= MAX_VISION_LINEAR_SPEED_MPS
          && omegaDegPerSec <= MAX_VISION_OMEGA_DEGPS;

      sbVisionPoseErrorM.setDouble(translationError);
      sbVisionRotErrorDeg.setDouble(rotationErrorDeg);
      sbVisionRobotSpeed.setDouble(robotSpeed);
      sbVisionOmegaDegPerSec.setDouble(omegaDegPerSec);
      sbVisionAccepted.setBoolean(accept);

      if (accept) {
        drivetrain.addVisionMeasurement(update.pose(), update.timestampSeconds(), update.stdDevs());
      }
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

  private static class CenterYawResult {
    final double yawRad;
    final int tagsUsed;

    CenterYawResult(double yawRad, int tagsUsed) {
      this.yawRad = yawRad;
      this.tagsUsed = tagsUsed;
    }
  }

  private static double getCorrectedYaw(double rawYaw, double rawDist) {
    
    boolean flip = false;
    if (rawYaw % Math.PI != rawYaw) {
      rawYaw = -rawYaw;
      flip = true;
    }

    final double xTrans = Constants.VisionConstants.CAM_X_METERS;
    final double yTrans = Constants.VisionConstants.CAM_Y_METERS;
    final double fullTrans = Math.hypot(xTrans, yTrans);

    double futureDist = Math.sqrt(Math.pow(rawDist,2) + Math.pow(fullTrans,2) - 2 * rawDist * fullTrans * Math.cos(Units.degreesToRadians(rawYaw)));
    double yawCorrected = (flip ? -rawYaw : rawYaw) + Units.radiansToDegrees(Math.acos((-Math.pow(fullTrans,2) + Math.pow(futureDist,2) + Math.pow(rawDist, 2)) / (2 * futureDist * rawDist)));

    return yawCorrected;
  }

  // we replaced this offset in the main thing
  private double getRobotCenterYawRad(PhotonTrackedTarget t) {
    double xCam = t.getBestCameraToTarget().getX();
    double yCam = t.getBestCameraToTarget().getY();

    double xRobot = xCam + Constants.VisionConstants.CAM_X_METERS;
    double yRobot = yCam + Constants.VisionConstants.CAM_Y_METERS;

    return Math.atan2(yRobot, xRobot);
  }

  // I know this isn't very organized but you get the idea hopefully
  public static class CenterFullCalcResult {
    final double dist;
    final double forward;
    final double lateral;
    final double yawRad;
    final int tagsUsed;

    CenterFullCalcResult (double dist, double forward, double lateral, double yawRad, int tagsUsed) {
      this.dist = dist;
      this.forward = forward;
      this.lateral = lateral;
      this.yawRad = yawRad;
      this.tagsUsed = tagsUsed;
    }

    private CenterYawResult toCenterYawResult() {
      return new CenterYawResult(yawRad, tagsUsed);
    }

    public boolean isValid(){
      return tagsUsed != 0;
    }

    public double getDist(){  return dist; }
    public double getForward(){  return forward; }
    public double getLateral(){  return lateral; }
    public double getYawRad(){  return yawRad; }
    public double gettagsUsed(){  return tagsUsed; }
  }

  public static CenterFullCalcResult calcHubCenterDistAndYaw(PhotonCamera camera) {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return new CenterFullCalcResult(0.0, 0.0,0.0, 0.0, 0);

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
    double rawDist = 0.0;
    int tagCount = 0;
    // remove non-hub targets
    List<PhotonTrackedTarget> targets = res.getTargets().stream().filter((t) -> hubIDs.contains(t.getFiducialId())).collect(Collectors.toList());

    if (targets.size() == 1) {
      var t = targets.get(0);
      yawTotal += t.getYaw();
      rawDist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
      tagCount++;
    } else if (targets.size() == 2) {
      var t0 = targets.get(0);
      var t1 = targets.get(1);

      if ((t1.getFiducialId() - t0.getFiducialId()) == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)
          || !(hubCenters.contains(t0.getFiducialId()) || hubCenters.contains(t1.getFiducialId()))) {
        for (var t : targets) {
          yawTotal += t.getYaw();
          rawDist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
          tagCount++;
        }
      } else {
        for (var t : targets) {
          if (hubCenters.contains(t.getFiducialId())) {
            yawTotal += t.getYaw();
            rawDist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
            tagCount++;
          }
        }
      }
    } else if (targets.size() == 3) {
      for (var t : targets) {
        yawTotal += t.getYaw();
        rawDist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
        tagCount++;
      }
    } else {
      for (var t : targets) {
        yawTotal += t.getYaw();
        rawDist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
        tagCount++;
      }
    }

    if (tagCount == 0) return new CenterFullCalcResult(0.0, 0.0,0.0, 0.0, 0);


    double rawYaw = yawTotal / tagCount;
    rawDist /= tagCount;
    

    // Yaw Correction (more inaccurate because it uses rawDist)
    //   not using the helper function because I also need futureDist
    boolean flip = false;
    if (rawYaw % Math.PI != rawYaw) {
      rawYaw = -rawYaw;
      flip = true;
    }

    final double xTrans = Constants.VisionConstants.CAM_X_METERS;
    final double yTrans = Constants.VisionConstants.CAM_Y_METERS;
    final double fullTrans = Math.hypot(xTrans, yTrans);

    double futureDist = Math.sqrt(Math.pow(rawDist,2) + Math.pow(fullTrans,2) - 2 * rawDist * fullTrans * Math.cos(Units.degreesToRadians(rawYaw)));
    double yawCorrected = (flip ? -rawYaw : rawYaw) + Units.radiansToDegrees(Math.acos((-Math.pow(fullTrans,2) + Math.pow(futureDist,2) + Math.pow(rawDist, 2)) / (2 * futureDist * rawDist)));
    
    // Dist Correction (more accureate because it's uses yawCorrected)
    
    final double hubCenterToEdge = Units.inchesToMeters(23.5); // in meters (I hope that is photon vision's translation unit)
    final double hubCenterToOffsetTag = Units.inchesToMeters(Math.sqrt(748.25)); //-------------------------------------------------------------------
    if (tagCount >= 2) {
      PhotonTrackedTarget t0 = targets.get(0);
      PhotonTrackedTarget t1 = targets.get(1);

      double t0Dist = Math.hypot(t0.getBestCameraToTarget().getX(), t0.getBestCameraToTarget().getY());
      double t1Dist = Math.hypot(t1.getBestCameraToTarget().getX(), t1.getBestCameraToTarget().getY());
      
      // Maps center ID to it's adjacent ID
      final Map<Integer, Integer> centerIdAdjacencies = blueAlliance ? Map.of(26,25,21,24,20,19,18,27) : Map.of(10,9,5,8,4,3,2,11);

      double forwardDist;
      double lateralDist;
      if (centerIdAdjacencies.get(t0.getFiducialId()) == t1.getFiducialId() || centerIdAdjacencies.get(t1.getFiducialId()) == t0.getFiducialId()) {
        final double hubTagToTag = Units.inchesToMeters(14); // in meters

        double centerAng = 90 + (180 - Math.abs(t0.getYaw() - t1.getYaw()) - (hubCenters.contains(t0.getFiducialId()) ? Math.acos((Math.pow(t1Dist,2) + Math.pow(hubTagToTag,2) - Math.pow(t0Dist,2)) / (2 * t1Dist * hubTagToTag)) :
         Math.acos((Math.pow(t0Dist,2) + Math.pow(hubTagToTag,2) - Math.pow(t1Dist,2)) / (2 * t0Dist * hubTagToTag))));

        // if greater than 180, subtract from 360
        if (centerAng > 180) {  centerAng -= 360; }
        
        // law of cos
        futureDist = Math.sqrt(Math.pow(hubCenterToEdge,2) + Math.pow((hubCenters.contains(t0.getFiducialId()) ? t0Dist : t1Dist),2) - 2 * hubCenterToEdge * (hubCenters.contains(t0.getFiducialId()) ? t0Dist : t1Dist) * Math.cos(Units.degreesToRadians(360-centerAng)));
        // law of sines
        yawCorrected += Units.radiansToDegrees(Math.asin(hubCenterToEdge * Math.sin(Units.degreesToRadians(centerAng))/futureDist));

        //Uses rotation math but treats the center as (0, futureDist)
        double robotAngle = getCorrectedYaw(t0.getYaw(), Math.hypot(t0.getBestCameraToTarget().getX(), t0.getBestCameraToTarget().getY())) + Math.acos((Math.pow(t0Dist, 2) + Math.pow(futureDist, 2) - Math.pow(hubCenterToEdge,2)) / (2 * t0Dist * futureDist));
        forwardDist = -futureDist * Math.sin(robotAngle);
        lateralDist = futureDist * Math.cos(robotAngle);
      } else {
        // This should work for the other case too but it is longer
        double hubTagToTag;
        double t0ToCenter;
        double hubCenterAngle;
        if (hubCenters.contains(t0.getFiducialId()) || hubCenters.contains(t1.getFiducialId())){
          hubTagToTag = Units.inchesToMeters(Math.sqrt(642.5)); // same as Math.hypot(9.5,23.5) but in meters
          t0ToCenter = hubCenters.contains(t0.getFiducialId()) ? hubCenterToEdge : hubCenterToOffsetTag;
          hubCenterAngle = hubCenters.contains(t0.getFiducialId()) ? 
          Math.acos((Math.pow(Units.inchesToMeters(23.5),2) + Math.pow(hubTagToTag,2) - Math.pow(hubCenterToOffsetTag,2))/(2*Units.inchesToMeters(23.5)*hubTagToTag))
          : Math.acos((Math.pow(hubTagToTag,2) + Math.pow(hubCenterToOffsetTag,2) - Math.pow(Units.inchesToMeters(23.5),2))/(2*hubTagToTag*hubCenterToOffsetTag));
        } else {
          hubTagToTag = Units.inchesToMeters(9.5 * Math.sqrt(2)); // same as Math.hypot(9.5,9.5) but in meters
          t0ToCenter = hubCenterToOffsetTag;
          hubCenterAngle = Math.pow(hubTagToTag,2)/(2*hubTagToTag*hubCenterToOffsetTag);
        }

        final double tagCenterAngle = Math.acos((Math.pow(hubTagToTag,2) + Math.pow(t0Dist,2) - Math.pow(t1Dist,2))/(2 * hubTagToTag * t0Dist));
        final double extCenterAngle = 180 - tagCenterAngle - hubCenterAngle;

        lateralDist = t0ToCenter * Math.sin(extCenterAngle);
        forwardDist = t0ToCenter * Math.cos(extCenterAngle) + t0Dist;

        futureDist = Math.hypot(forwardDist, lateralDist);

        // lateralDist and forwardDist do not account for the the angle of the robot towards the tags yet
        final double robotAngle = Units.degreesToRadians(getCorrectedYaw(t0.getYaw(), Math.hypot(t0.getBestCameraToTarget().getX(), t0.getBestCameraToTarget().getY())));
        double temp = forwardDist * Math.cos(robotAngle) - lateralDist * Math.sin(robotAngle);
        lateralDist = forwardDist * Math.sin(robotAngle) + lateralDist * Math.cos(robotAngle);
        forwardDist = temp;
      }
      return new CenterFullCalcResult(futureDist, forwardDist, lateralDist, yawCorrected, tagCount);
    }

    return new CenterFullCalcResult(futureDist, targets.get(0).getBestCameraToTarget().getX(), targets.get(0).getBestCameraToTarget().getY(), yawCorrected, tagCount);
  } 

  private CenterYawResult calcHubCenterYawRadWithCount() {
    return calcHubCenterDistAndYaw(camera).toCenterYawResult();
  }

  public double calcAimOmegaRadPerSec() {
    CenterYawResult center = calcHubCenterYawRadWithCount();
    if (center.tagsUsed > 0) {
      return -yawPid.calculate(center.yawRad, 0.0);
    }

    return getBestTarget()
        .map(t -> -yawPid.calculate(getRobotCenterYawRad(t), 0.0))
        .orElse(0.0);
  }

  public double calcStrafeVyRobotMps() {
    return getBestTarget().map(t -> {
      double lateral =
          t.getBestCameraToTarget().getTranslation().getY()
              + Constants.VisionConstants.CAM_Y_METERS;
      double vy = strafePid.calculate(lateral, 0.0);
      if (Math.abs(lateral) < strafePid.getErrorTolerance()) vy = 0.0;
      return Math.max(-1.5, Math.min(1.5, vy));
    }).orElse(0.0);
  }

  public double calcApproachVxRobotMps(double stopDistanceMeters, double approachSpeedMps) {
    return getBestTarget().map(t -> {
      double x =
          t.getBestCameraToTarget().getTranslation().getX()
              + Constants.VisionConstants.CAM_X_METERS;
      return Math.abs(x) > stopDistanceMeters ? +approachSpeedMps : 0.0;
    }).orElse(0.0);
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
      double xCam = t.getBestCameraToTarget().getTranslation().getX();
      double yCam = t.getBestCameraToTarget().getTranslation().getY();
      double xRobot = xCam + Constants.VisionConstants.CAM_X_METERS;
      double yRobot = yCam + Constants.VisionConstants.CAM_Y_METERS;

      sbTagId.setDouble(t.getFiducialId());
      sbYawDeg.setDouble(t.getYaw());
      sbDistX.setDouble(xCam);
      sbDistY.setDouble(yCam);
      sbRobotDistX.setDouble(xRobot);
      sbRobotDistY.setDouble(yRobot);
    } else {
      sbTagId.setDouble(-1);
      sbYawDeg.setDouble(0.0);
      sbDistX.setDouble(0.0);
      sbDistY.setDouble(0.0);
      sbRobotDistX.setDouble(0.0);
      sbRobotDistY.setDouble(0.0);
    }

    CenterYawResult center = calcHubCenterYawRadWithCount();
    sbCenterYawDeg.setDouble(Units.radiansToDegrees(center.yawRad));
    sbCenterTagsUsed.setDouble(center.tagsUsed);
  }
}