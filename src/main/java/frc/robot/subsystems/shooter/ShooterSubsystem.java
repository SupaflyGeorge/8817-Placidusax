package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooterhood.HoodIO;
import frc.robot.subsystems.shooterhood.HoodIOTalonFX;

@Logged
public class ShooterSubsystem extends SubsystemBase {

  public enum WantedState { IDLE, PREPARE_SHOT, SHOOTING, MANUAL_SHOT, FAR_SHOT  }

  public static class TagAimData {
    public final double distanceM;
    public final double forwardM;
    public final double lateralM;

    public TagAimData(double d, double f, double l) {
      distanceM = d;
      forwardM = f;
      lateralM = l;
    }

    public boolean isValid() { return distanceM > 0.0; }
  }

  private final ShooterIO shooterIO = new ShooterIOTalonFX();
  private final ShooterIO.ShooterIOInputs shooterInputs = new ShooterIO.ShooterIOInputs();
  private final HoodIO hoodIO = new HoodIOTalonFX();
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();

  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

  private WantedState wanted = WantedState.IDLE;

  private double targetTopRps;
  private double targetBottomRps;
  private double targetFeedRps;
  private double targetHoodRot;

  private double mapRps;
  private double mapHoodRot;

  private boolean feedEnabled = false;

  private double manualHoodRot = 0.0;
  private double manualTopRps = Constants.ShooterConstants.SHOOTER_TOP_RPS;
  private double manualBottomRps = Constants.ShooterConstants.SHOOTER_BOTTOM_RPS;

  private double shooterRpsOffset = 0.0;
  private double shotDistanceOverrideM = -1.0;

  private static final double READY_TOL_RPS = Constants.ShooterConstants.READY_TOL_RPS;
  private static final double READY_MIN_RPS = Constants.ShooterConstants.READY_MIN_RPS;

  public ShooterSubsystem() {
    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
    SmartDashboard.putNumber("Shooter/RpsOffset", 0.0);
    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);
    if (Constants.ShooterConstants.LIVE_TUNING) {
      SmartDashboard.putNumber("Shooter/FeederRPS", Constants.ShooterConstants.FEEDER_RPS);
    }
  }

  public void setWantedState(WantedState state) {
    wanted = state;
    if (state != WantedState.SHOOTING && state != WantedState.MANUAL_SHOT) {
      feedEnabled = false;
    }
  }

  public WantedState getWantedState() {
    return wanted;
  }

  public void setFeedEnabled(boolean enable) {
    feedEnabled = enable;
  }

  public void setShotDistanceOverrideMeters(double distanceM) {
    shotDistanceOverrideM = distanceM;
  }

  public void clearShotDistanceOverride() {
    shotDistanceOverrideM = -1.0;
  }

  public boolean hasShotDistanceOverride() {
    return shotDistanceOverrideM > 0.0;
  }

  public void syncManualToCurrentTargets() {
    manualHoodRot = clamp(
        hoodInputs.hoodPositionRot,
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Constants.ShooterConstants.HOOD_MAX_ROT);
    manualTopRps = clamp(
        targetTopRps,
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Constants.ShooterConstants.SHOOTER_MAX_RPS);
    manualBottomRps = clamp(
        targetBottomRps,
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Constants.ShooterConstants.SHOOTER_MAX_RPS);

    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);
    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
  }

  public void adjustManualHood(double deltaRot) {
    manualHoodRot = clamp(
        manualHoodRot + deltaRot,
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Constants.ShooterConstants.HOOD_MAX_ROT);
    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);
  }

  public void adjustManualShooterRps(double deltaRps) {
    manualTopRps = clamp(
        manualTopRps + deltaRps,
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Constants.ShooterConstants.SHOOTER_MAX_RPS);
    manualBottomRps = clamp(
        manualBottomRps + deltaRps,
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Constants.ShooterConstants.SHOOTER_MAX_RPS);

    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
  }

  public void increaseShooterSpeed() {
    shooterRpsOffset += 1.0;
    SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset);
  }

  public void decreaseShooterSpeed() {
    shooterRpsOffset -= 1.0;
    SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset);
  }

  public void resetShooterOffset() {
    shooterRpsOffset = 0.0;
    SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset);
  }

  public double getShooterOffset() {
    return shooterRpsOffset;
  }

  @Logged(importance = Importance.CRITICAL)
public boolean flywheelsAtSpeed() {
  double avgTarget = (Math.abs(targetTopRps) + Math.abs(targetBottomRps)) * 0.5;
  if (avgTarget < READY_MIN_RPS) return false;

  double top = Math.abs(shooterInputs.topVelocityRps);
  double bottom = Math.abs(shooterInputs.bottomVelocityRps);

  boolean withinTolerance =
      Math.abs(top - Math.abs(targetTopRps)) <= READY_TOL_RPS &&
      Math.abs(bottom - Math.abs(targetBottomRps)) <= READY_TOL_RPS;

  
  double minPercent = 0.93;
  boolean abovePercent =
      top >= Math.abs(targetTopRps) * minPercent &&
      bottom >= Math.abs(targetBottomRps) * minPercent;

  return withinTolerance || abovePercent;
}
  @Logged(importance = Importance.CRITICAL)
  public boolean hoodAtTarget() {
    return Math.abs(hoodInputs.hoodPositionRot - targetHoodRot)
        <= Constants.ShooterConstants.HOOD_READY_TOL_ROT;
  }

  @NotLogged
  public TagAimData getHubTagAimData() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return new TagAimData(-1.0, -1.0, 0.0);

    boolean blue =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;
    List<Integer> hubIds = blue ? List.of(21, 24, 25, 26, 18, 27) : List.of(2, 11, 8, 5, 9, 10);
    List<Integer> hubCenters = blue ? List.of(18, 21, 26) : List.of(2, 5, 10);
    
    double camX = Constants.VisionConstants.CAM_X_METERS;
    double camY = Constants.VisionConstants.CAM_Y_METERS;

    double dist = 0.0;
    double forward = 0.0;
    double lateral = 0.0;
    int tagCount = 0;
    
    var targets = res.getTargets();

    // Strategy depends on how many hub tags we see
    if (targets.size() == 1) {
      var t = targets.get(0);
      if (hubIds.contains(t.getFiducialId())) { dist += Units.degreesToRadians(t.getYaw()); tagCount++; }
    } else if (targets.size() == 2) {
      var t0 = targets.get(0); var t1 = targets.get(1);
      if ((t1.getFiducialId() - t0.getFiducialId()) == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)
          || !(hubCenters.contains(t0.getFiducialId()) || hubCenters.contains(t1.getFiducialId()))) {
        for (var t : targets) { 
          if (hubIds.contains(t.getFiducialId())) { 
            forward += t.getBestCameraToTarget().getX() + camX;
            lateral += t.getBestCameraToTarget().getY() + camY;
            dist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
            tagCount++;
          } }
      } else {
        for (var t : targets) {
          if (hubCenters.contains(t.getFiducialId())) {
            forward += t.getBestCameraToTarget().getX() + camX;
            lateral += t.getBestCameraToTarget().getY() + camY;
            dist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
            tagCount++;
          }
        }
      }
    } else if (targets.size() == 3) {
      for (var t : targets) {
        if (hubCenters.contains(t.getFiducialId())) {
          forward += t.getBestCameraToTarget().getX() + camX;
          lateral += t.getBestCameraToTarget().getY() + camY;
          dist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
          tagCount++;
        }
      }
    } else {
      for (var t : targets) {
        if (hubIds.contains(t.getFiducialId())) {
          forward += t.getBestCameraToTarget().getX() + camX;
          lateral += t.getBestCameraToTarget().getY() + camY;
          dist += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
          tagCount++;
        }
      }
    }
    


    if (tagCount == 0) return new TagAimData(-1.0, -1.0, 0.0);
    return new TagAimData(dist / tagCount, forward / tagCount, lateral / tagCount);
  }

  @NotLogged
  public boolean hasHubTarget() {
    return getHubTagAimData().isValid();
  }

  @NotLogged
  public double getDistanceToTagMeters() {
    return getHubTagAimData().distanceM;
  }

  @NotLogged
  public double getForwardToTagMeters() {
    return getHubTagAimData().forwardM;
  }

  @NotLogged
  public double getLateralToTagMeters() {
    return getHubTagAimData().lateralM;
  }

  private double interpolate(double x, double[] xTable, double[] yTable) {
    if (x <= xTable[0]) return yTable[0];
    if (x >= xTable[xTable.length - 1]) return yTable[yTable.length - 1];

    for (int i = 0; i < xTable.length - 1; i++) {
      if (x >= xTable[i] && x <= xTable[i + 1]) {
        double ratio = (x - xTable[i]) / (xTable[i + 1] - xTable[i]);
        return yTable[i] + ratio * (yTable[i + 1] - yTable[i]);
      }
    }

    return yTable[yTable.length - 1];
  }

  public double calcMapHoodForDistance(double dist) {
    if (dist < 0.0) return hoodInputs.hoodPositionRot;
    return clamp(
        interpolate(
            dist,
            Constants.ShooterConstants.SHOT_DISTANCE_M,
            Constants.ShooterConstants.SHOT_HOOD_ROT),
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Constants.ShooterConstants.HOOD_MAX_ROT);
  }

  public double calcMapRpsForDistance(double dist) {
    if (dist < 0.0) return Constants.ShooterConstants.SHOOTER_TOP_RPS;
    return clamp(
        interpolate(
            dist,
            Constants.ShooterConstants.SHOT_DISTANCE_M,
            Constants.ShooterConstants.SHOT_RPS),
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Constants.ShooterConstants.SHOOTER_MAX_RPS);
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    hoodIO.updateInputs(hoodInputs);

    TagAimData aim = getHubTagAimData();
    double rawDist = aim.distanceM;
    double shotDist = shotDistanceOverrideM > 0.0 ? shotDistanceOverrideM : rawDist;

    mapHoodRot = calcMapHoodForDistance(shotDist);
    mapRps = calcMapRpsForDistance(shotDist);

    switch (wanted) {
      case IDLE -> {
        targetTopRps = 0.0;
        targetBottomRps = 0.0;
        targetFeedRps = 0.0;
        targetHoodRot = 0.0;
        feedEnabled = false;
      }

      case PREPARE_SHOT, SHOOTING -> {
        targetHoodRot = mapHoodRot;
        targetTopRps = mapRps + shooterRpsOffset;
        targetBottomRps = mapRps + shooterRpsOffset;

        if (wanted == WantedState.SHOOTING) {
          double feed = Constants.ShooterConstants.FEEDER_RPS;
          if (Constants.ShooterConstants.LIVE_TUNING) {
            feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
          }
          targetFeedRps = feedEnabled ? feed : 0.0;
        } else {
          targetFeedRps = 0.0;
        }
      }

      case MANUAL_SHOT -> {
        targetHoodRot = manualHoodRot;
        targetTopRps = manualTopRps + shooterRpsOffset;
        targetBottomRps = manualBottomRps + shooterRpsOffset;

        double feed = Constants.ShooterConstants.FEEDER_RPS;
        if (Constants.ShooterConstants.LIVE_TUNING) {
          feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
        }
        targetFeedRps = feedEnabled ? feed : 0.0;
      }
      case FAR_SHOT -> {
        targetHoodRot = manualHoodRot;
        targetTopRps = 60;
        targetBottomRps = 60;

        double feed = Constants.ShooterConstants.FEEDER_RPS;
        if (Constants.ShooterConstants.LIVE_TUNING) {
          feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
        }
        targetFeedRps = feedEnabled ? feed : 0.0;
      }

    }

    
    

    if (wanted == WantedState.IDLE) {
      shooterIO.stop();
    } else {
      shooterIO.setFlywheelVelocityRps(
          targetTopRps,
          targetBottomRps,
          targetFeedRps);
    }

    hoodIO.setHoodPositionRot(targetHoodRot);

    SmartDashboard.putNumber("Vision/TagDistanceM", rawDist);
    SmartDashboard.putNumber("Vision/TagForwardM", aim.forwardM);
    SmartDashboard.putNumber("Vision/TagLateralM", aim.lateralM);

    SmartDashboard.putNumber("Shooter/ShotDistanceUsedM", shotDist);
    SmartDashboard.putBoolean("Shooter/UsingDistanceOverride", shotDistanceOverrideM > 0.0);

    SmartDashboard.putNumber("Shooter/TargetTopRPS", targetTopRps);
    SmartDashboard.putNumber("Shooter/TargetMidRPS", shooterInputs.midVelocityRps);
    SmartDashboard.putNumber("Shooter/TargetBottomRPS", targetBottomRps);

    SmartDashboard.putBoolean("Shooter/AtSpeed", flywheelsAtSpeed());
    SmartDashboard.putNumber("Shooter/MapBaseRPS", mapRps);
    SmartDashboard.putBoolean("Shooter/ManualShotActive", wanted == WantedState.MANUAL_SHOT);

    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);

    SmartDashboard.putNumber("Shooter/ActualTopRPS", shooterInputs.topVelocityRps);
    SmartDashboard.putNumber("Shooter/ActualMidRPS", shooterInputs.midVelocityRps);
    SmartDashboard.putNumber("Shooter/ActualBottomRPS", shooterInputs.bottomVelocityRps);
    SmartDashboard.putNumber("Shooter/ActualFeederRPS", shooterInputs.feederVelocityRps);

    SmartDashboard.putNumber("Hood/PositionDeg", hoodInputs.hoodPositionRot * 360.0);
    SmartDashboard.putNumber("Hood/TargetDeg", targetHoodRot * 360.0);
    SmartDashboard.putBoolean("Hood/AtTarget", hoodAtTarget());
    SmartDashboard.putNumber("Hood/MapBaseRot", mapHoodRot);
    SmartDashboard.putNumber("Hood/ManualRot", manualHoodRot);
  }

  @Override
  public void simulationPeriodic() {
    shooterIO.simulationPeriodic();
    hoodIO.simulationPeriodic();
  }
}