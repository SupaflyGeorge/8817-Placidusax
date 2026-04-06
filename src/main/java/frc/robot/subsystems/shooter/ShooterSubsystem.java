package frc.robot.subsystems.shooter;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooterhood.HoodIO;
import frc.robot.subsystems.shooterhood.HoodIOTalonFX;

/**
 * The shooter subsystem manages three things:
 *   1. Dual flywheels (top + bottom) for launching game pieces
 *   2. A feeder wheel that pushes pieces into the flywheels
 *   3. An adjustable hood that changes the launch angle
 *
 * It also reads AprilTag distance from the camera to auto-calculate
 * the correct flywheel speed and hood angle using lookup tables.
 *
 * State machine:
 *   IDLE         - everything off
 *   PREPARE_SHOT - spin flywheels + set hood based on distance, don't feed
 *   SHOOTING     - same as PREPARE_SHOT but feeds when feedEnabled is true
 *   MANUAL_SHOT  - uses operator-tuned speed/hood instead of the lookup table
 */
@Logged
public class ShooterSubsystem extends SubsystemBase {

  public enum WantedState { IDLE, PREPARE_SHOT, SHOOTING, MANUAL_SHOT }

  /** Holds distance + direction data from a visible hub AprilTag. */
  public static class TagAimData {
    public final double distanceM;
    public final double forwardM;
    public final double lateralM;

    public TagAimData(double d, double f, double l) {
      distanceM = d; forwardM = f; lateralM = l;
    }
    public boolean isValid() { return distanceM > 0.0; }
  }

  // Hardware IO layers
  private final ShooterIO shooterIO = new ShooterIOTalonFX();
  private final ShooterIO.ShooterIOInputs shooterInputs = new ShooterIO.ShooterIOInputs();
  private final HoodIO hoodIO = new HoodIOTalonFX();
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();

  // Camera for reading tag distance (used for auto-aim shot calculations)
  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

  private WantedState wanted = WantedState.IDLE;

  // Current targets being commanded to the hardware
  private double targetTopRps;
  private double targetBottomRps;
  private double targetFeedRps;
  private double targetHoodRot;

  // Values from the distance lookup table (before offset)
  private double mapRps;
  private double mapHoodRot;

  // Feed gate: flywheels spin up first, then we set this true to push the piece in
  private boolean feedEnabled = false;

  // Manual fallback values (operator-tunable via d-pad)
  private double manualHoodRot = 0.0;
  private double manualTopRps = Constants.ShooterConstants.SHOOTER_TOP_RPS;
  private double manualBottomRps = Constants.ShooterConstants.SHOOTER_BOTTOM_RPS;

  // Live offset the driver can bump up/down with Y and d-pad
  private double shooterRpsOffset = 0.0;

  // Optional: override the tag distance (e.g. for testing from a fixed spot)
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

  // --- State management ---

  public void setWantedState(WantedState state) {
    wanted = state;
    if (state != WantedState.SHOOTING && state != WantedState.MANUAL_SHOT) feedEnabled = false;
  }
  public WantedState getWantedState() { return wanted; }
  public void setFeedEnabled(boolean enable) { feedEnabled = enable; }

  // --- Distance override (for testing) ---
  public void setShotDistanceOverrideMeters(double distanceM) { shotDistanceOverrideM = distanceM; }
  public void clearShotDistanceOverride() { shotDistanceOverrideM = -1.0; }
  public boolean hasShotDistanceOverride() { return shotDistanceOverrideM > 0.0; }

  /** Copy current auto-aim values into the manual fallback so operator can tweak from there. */
  public void syncManualToCurrentTargets() {
    manualHoodRot = clamp(targetHoodRot, Constants.ShooterConstants.HOOD_MIN_ROT, Constants.ShooterConstants.HOOD_MAX_ROT);
    manualTopRps = clamp(targetTopRps, Constants.ShooterConstants.SHOOTER_MIN_RPS, Constants.ShooterConstants.SHOOTER_MAX_RPS);
    manualBottomRps = clamp(targetBottomRps, Constants.ShooterConstants.SHOOTER_MIN_RPS, Constants.ShooterConstants.SHOOTER_MAX_RPS);
    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);
    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
  }

  /** Operator d-pad: nudge hood angle up or down. */
  public void adjustManualHood(double deltaRot) {
    manualHoodRot = clamp(manualHoodRot + deltaRot, Constants.ShooterConstants.HOOD_MIN_ROT, Constants.ShooterConstants.HOOD_MAX_ROT);
    SmartDashboard.putNumber("Hood/ManualTargetRot", manualHoodRot);
  }

  /** Operator d-pad: nudge flywheel speed up or down. */
  public void adjustManualShooterRps(double deltaRps) {
    manualTopRps = clamp(manualTopRps + deltaRps, Constants.ShooterConstants.SHOOTER_MIN_RPS, Constants.ShooterConstants.SHOOTER_MAX_RPS);
    manualBottomRps = clamp(manualBottomRps + deltaRps, Constants.ShooterConstants.SHOOTER_MIN_RPS, Constants.ShooterConstants.SHOOTER_MAX_RPS);
    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
  }

  // Driver Y / d-pad: live speed offset
  public void increaseShooterSpeed() { shooterRpsOffset += 1.0; SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset); }
  public void decreaseShooterSpeed() { shooterRpsOffset -= 1.0; SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset); }
  public void resetShooterOffset() { shooterRpsOffset = 0.0; SmartDashboard.putNumber("Shooter/RpsOffset", shooterRpsOffset); }
  public double getShooterOffset() { return shooterRpsOffset; }

  // --- Readiness checks ---

  /** True when both flywheels are within tolerance of their target speed. */
  @Logged(importance = Importance.CRITICAL)
  public boolean flywheelsAtSpeed() {
    double avg = (Math.abs(targetTopRps) + Math.abs(targetBottomRps)) * 0.5;
    if (avg < READY_MIN_RPS) return false;
    return Math.abs(shooterInputs.topVelocityRps - targetTopRps) <= READY_TOL_RPS
        && Math.abs(shooterInputs.bottomVelocityRps - targetBottomRps) <= READY_TOL_RPS;
  }

  /** True when hood is within tolerance of its target angle. */
  @Logged(importance = Importance.CRITICAL)
  public boolean hoodAtTarget() {
    return Math.abs(hoodInputs.hoodPositionRot - targetHoodRot) <= Constants.ShooterConstants.HOOD_READY_TOL_ROT;
  }

  // --- Tag aiming ---

  /**
   * Reads the camera and returns averaged distance/position data for hub AprilTags.
   * Filters by alliance-specific tag IDs so we only aim at our own hub.
   */
  @NotLogged
  public TagAimData getHubTagAimData() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return new TagAimData(-1.0, -1.0, 0.0);

    boolean blue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    List<Integer> hubIds = blue ? List.of(21, 24, 25, 26, 18, 27) : List.of(2, 11, 8, 5, 9, 10);

    double dist = 0.0, forward = 0.0, lateral = 0.0;
    int count = 0;
    for (PhotonTrackedTarget t : res.getTargets()) {
      if (hubIds.contains(t.getFiducialId())) {
        double x = t.getBestCameraToTarget().getX();
        double y = t.getBestCameraToTarget().getY();
        dist += Math.hypot(x, y);
        forward += x;
        lateral += y;
        count++;
      }
    }
    if (count == 0) return new TagAimData(-1.0, -1.0, 0.0);
    return new TagAimData(dist / count, forward / count, lateral / count);
  }

  @NotLogged public boolean hasHubTarget() { return getHubTagAimData().isValid(); }
  @NotLogged public double getDistanceToTagMeters() { return getHubTagAimData().distanceM; }
  @NotLogged public double getForwardToTagMeters() { return getHubTagAimData().forwardM; }
  @NotLogged public double getLateralToTagMeters() { return getHubTagAimData().lateralM; }

  // --- Lookup table interpolation ---

  /**
   * Linear interpolation between measured data points.
   * Given a distance, find the corresponding hood angle or flywheel speed
   * by blending between the two nearest entries in the table.
   */
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

  /** Look up the correct hood angle for a given distance. */
  public double calcMapHoodForDistance(double dist) {
    if (dist < 0.0) return hoodInputs.hoodPositionRot;
    return clamp(interpolate(dist, Constants.ShooterConstants.SHOT_DISTANCE_M, Constants.ShooterConstants.SHOT_HOOD_ROT),
        Constants.ShooterConstants.HOOD_MIN_ROT, Constants.ShooterConstants.HOOD_MAX_ROT);
  }

  /** Look up the correct flywheel RPS for a given distance. */
  public double calcMapRpsForDistance(double dist) {
    if (dist < 0.0) return Constants.ShooterConstants.SHOOTER_TOP_RPS;
    return clamp(interpolate(dist, Constants.ShooterConstants.SHOT_DISTANCE_M, Constants.ShooterConstants.SHOT_RPS),
        Constants.ShooterConstants.SHOOTER_MIN_RPS, Constants.ShooterConstants.SHOOTER_MAX_RPS);
  }

  private static double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }

  // --- Main loop ---

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    hoodIO.updateInputs(hoodInputs);

    // Get current distance to hub (or use override if set)
    TagAimData aim = getHubTagAimData();
    double rawDist = aim.distanceM;
    double shotDist = shotDistanceOverrideM > 0.0 ? shotDistanceOverrideM : rawDist;

    // Look up table values for this distance
    mapHoodRot = calcMapHoodForDistance(shotDist);
    mapRps = calcMapRpsForDistance(shotDist);

    // Apply state machine
    switch (wanted) {
      case IDLE -> {
        targetTopRps = 0.0; targetBottomRps = 0.0; targetFeedRps = 0.0; targetHoodRot = 0.0;
        feedEnabled = false;
      }
      case PREPARE_SHOT, SHOOTING -> {
        targetHoodRot = mapHoodRot;
        targetTopRps = mapRps + shooterRpsOffset;
        targetBottomRps = mapRps + shooterRpsOffset;
        if (wanted == WantedState.SHOOTING) {
          double feed = Constants.ShooterConstants.FEEDER_RPS;
          if (Constants.ShooterConstants.LIVE_TUNING) feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
          targetFeedRps = feedEnabled ? feed : 0.0;
        } else { targetFeedRps = 0.0; }
      }
      case MANUAL_SHOT -> {
        targetHoodRot = Constants.ShooterConstants.HOOD_MAX_ROT;
        targetTopRps = manualTopRps + shooterRpsOffset;
        targetBottomRps = manualBottomRps + shooterRpsOffset;
        double feed = Constants.ShooterConstants.FEEDER_RPS;
        if (Constants.ShooterConstants.LIVE_TUNING) feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
        targetFeedRps = feedEnabled ? feed : 0.0;
      }
    }

    // Send commands to hardware
    if (wanted == WantedState.IDLE) shooterIO.stop();
    else shooterIO.setFlywheelVelocityRps(targetTopRps, targetBottomRps, targetFeedRps);
    hoodIO.setHoodPositionRot(targetHoodRot);

    // Publish telemetry
    SmartDashboard.putNumber("Vision/TagDistanceM", rawDist);
    SmartDashboard.putNumber("Vision/TagForwardM", aim.forwardM);
    SmartDashboard.putNumber("Vision/TagLateralM", aim.lateralM);
    SmartDashboard.putNumber("Shooter/ShotDistanceUsedM", shotDist);
    SmartDashboard.putBoolean("Shooter/UsingDistanceOverride", shotDistanceOverrideM > 0.0);
    SmartDashboard.putNumber("Shooter/TargetTopRPS", targetTopRps);
    SmartDashboard.putNumber("Shooter/TargetBottomRPS", targetBottomRps);
    SmartDashboard.putBoolean("Shooter/AtSpeed", flywheelsAtSpeed());
    SmartDashboard.putNumber("Shooter/MapBaseRPS", mapRps);
    SmartDashboard.putBoolean("Shooter/ManualShotActive", wanted == WantedState.MANUAL_SHOT);
    SmartDashboard.putNumber("Shooter/ManualTopRPS", manualTopRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", manualBottomRps);
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
