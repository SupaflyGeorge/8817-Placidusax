package frc.robot.subsystems.shooter;

import java.util.ArrayList;
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

@Logged
public class ShooterSubsystem extends SubsystemBase {

  public enum WantedState {
    IDLE,
    PREPARE_SHOT,
    SHOOTING
  }

  private static class TagAimData {
    public final double distanceM;
    public final double forwardM;
    public final double lateralM;

    public TagAimData(double distanceM, double forwardM, double lateralM) {
      this.distanceM = distanceM;
      this.forwardM = forwardM;
      this.lateralM = lateralM;
    }
  }

  private final ShooterIO shooterIO = new ShooterIOTalonFX();
  private final ShooterIO.ShooterIOInputs shooterInputs = new ShooterIO.ShooterIOInputs();

  private final HoodIO hoodIO = new HoodIOTalonFX();
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();

  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

  private WantedState wanted = WantedState.IDLE;

  private double targetTopRps = 0.0;
  private double targetBottomRps = 0.0;
  private double targetFeedRps = 0.0;
  private double targetHoodRot = 0.0;

  private boolean feedEnabled = false;

  private boolean hoodManualMode = true;
  private double hoodManualRot = 0.0;

  private boolean shooterManualMode = false;
  private double shooterManualTopRps = 0.0;
  private double shooterManualBottomRps = 0.0;

  @SuppressWarnings("unused")
  private double filteredDistM = -1;

  private static final double READY_TOL_RPS = Constants.ShooterConstants.READY_TOL_RPS;
  private static final double READY_MIN_RPS = Constants.ShooterConstants.READY_MIN_RPS;

  public ShooterSubsystem() {
    SmartDashboard.putBoolean("Shooter/ManualSpeedMode", false);
    SmartDashboard.putNumber("Shooter/ManualTopRPS", Constants.ShooterConstants.SHOOTER_TOP_RPS);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", Constants.ShooterConstants.SHOOTER_BOTTOM_RPS);

    SmartDashboard.putBoolean("Hood/ManualTargetFromDashboard", false);
    SmartDashboard.putNumber("Hood/ManualTargetRot", 0.0);

    if (Constants.ShooterConstants.LIVE_TUNING) {
      SmartDashboard.putNumber("Shooter/TopRPS", Constants.ShooterConstants.SHOOTER_TOP_RPS);
      SmartDashboard.putNumber("Shooter/BottomRPS", Constants.ShooterConstants.SHOOTER_BOTTOM_RPS);
      SmartDashboard.putNumber("Shooter/FeederRPS", Constants.ShooterConstants.FEEDER_RPS);
    }
  }

  public void setWantedState(WantedState state) {
    wanted = state;

    if (state != WantedState.SHOOTING) {
      feedEnabled = false;
    }
  }

  public WantedState getWantedState() {
    return wanted;
  }

  public void setFeedEnabled(boolean enable) {
    feedEnabled = enable;
  }

  public void enableHoodManual(boolean enable) {
    hoodManualMode = enable;
    hoodManualRot = hoodInputs.hoodPositionRot;
  }

  public void adjustHoodManual(double deltaRot) {
    hoodManualRot += deltaRot;
    hoodManualRot =
        Math.max(
            Constants.ShooterConstants.HOOD_MIN_ROT,
            Math.min(Constants.ShooterConstants.HOOD_MAX_ROT, hoodManualRot));
  }

  public boolean isHoodManualMode() {
    return hoodManualMode;
  }

  public void setShooterManualMode(boolean enable) {
    shooterManualMode = enable;
  }

  public boolean isShooterManualMode() {
    return shooterManualMode;
  }

  public void setManualShooterSpeeds(double topRps, double bottomRps) {
    shooterManualTopRps = topRps;
    shooterManualBottomRps = bottomRps;
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean flywheelsAtSpeed() {
    double avg = (Math.abs(targetTopRps) + Math.abs(targetBottomRps)) * 0.5;
    if (avg < READY_MIN_RPS) return false;

    return (Math.abs(shooterInputs.topVelocityRps - targetTopRps) <= READY_TOL_RPS)
        && (Math.abs(shooterInputs.bottomVelocityRps - targetBottomRps) <= READY_TOL_RPS);
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean hoodAtTarget() {
    return Math.abs(hoodInputs.hoodPositionRot - targetHoodRot)
        <= Constants.ShooterConstants.HOOD_READY_TOL_ROT;
  }

  @NotLogged
  private TagAimData getTagAimData() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return new TagAimData(-1.0, -1.0, 0.0);

    List<Integer> hubCenters = new ArrayList<>();
    List<Integer> hubIDs = new ArrayList<>();

    boolean blueAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;

    if (blueAlliance) {
      hubCenters.addAll(List.of(18, 24, 26));
      hubIDs.addAll(List.of(21, 24, 19, 20, 18, 27));
    } else {
      hubCenters.addAll(List.of(2, 8, 10));
      hubIDs.addAll(List.of(2, 11, 8, 5, 9, 10));
    }

    double distTotal = 0.0;
    double forwardTotal = 0.0;
    double lateralTotal = 0.0;
    int tagCount = 0;

    var targets = res.getTargets();

    if (targets.size() == 1) {
      PhotonTrackedTarget t = targets.get(0);
      if (hubIDs.contains(t.getFiducialId())) {
        double x = t.getBestCameraToTarget().getX();
        double y = t.getBestCameraToTarget().getY();

        distTotal += Math.hypot(x, y);
        forwardTotal += x;
        lateralTotal += y;
        tagCount++;
      }
    } else if (targets.size() == 2) {
      var t0 = targets.get(0);
      var t1 = targets.get(1);

      if ((t1.getFiducialId() - t0.getFiducialId())
          == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)) {
        for (var t : targets) {
          double x = t.getBestCameraToTarget().getX();
          double y = t.getBestCameraToTarget().getY();

          distTotal += Math.hypot(x, y);
          forwardTotal += x;
          lateralTotal += y;
          tagCount++;
        }
      } else {
        for (var t : targets) {
          if (hubCenters.contains(t.getFiducialId())) {
            double x = t.getBestCameraToTarget().getX();
            double y = t.getBestCameraToTarget().getY();

            distTotal += Math.hypot(x, y);
            forwardTotal += x;
            lateralTotal += y;
            tagCount++;
          }
        }
      }
    } else if (targets.size() == 3) {
      for (var t : targets) {
        if (hubCenters.contains(t.getFiducialId())) {
          double x = t.getBestCameraToTarget().getX();
          double y = t.getBestCameraToTarget().getY();

          distTotal += Math.hypot(x, y);
          forwardTotal += x;
          lateralTotal += y;
          tagCount++;
        }
      }
    } else {
      for (var t : targets) {
        if (hubIDs.contains(t.getFiducialId())) {
          double x = t.getBestCameraToTarget().getX();
          double y = t.getBestCameraToTarget().getY();

          distTotal += Math.hypot(x, y);
          forwardTotal += x;
          lateralTotal += y;
          tagCount++;
        }
      }
    }

    if (tagCount == 0) return new TagAimData(-1.0, -1.0, 0.0);

    return new TagAimData(
        distTotal / tagCount,
        forwardTotal / tagCount,
        lateralTotal / tagCount);
  }

  @NotLogged
  public double getDistanceToTagMeters() {
    return getTagAimData().distanceM;
  }

  private double calcHoodRotFromDistance(double distM) {
    if (distM < 0) return 0.0;

    final double a = Constants.ShooterConstants.HOOD_QUADRATIC_A;
    final double b = Constants.ShooterConstants.HOOD_QUADRATIC_B;
    final double c = Constants.ShooterConstants.HOOD_QUADRATIC_C;

    double formula = a * Math.pow(distM, 2) + b * distM + c;

    return Math.max(
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Math.min(Constants.ShooterConstants.HOOD_MAX_ROT, formula));
  }

  private double calcTopRpsFromTagPosition(TagAimData aimData) {
    if (aimData.distanceM < 0) return Constants.ShooterConstants.SHOOTER_TOP_RPS;

    double distComponent =
        Constants.ShooterConstants.SHOOTER_TOP_RPS
            + (aimData.distanceM * Constants.ShooterConstants.TOP_RPS_PER_METER);

    double lateralComponent =
        Math.abs(aimData.lateralM) * Constants.ShooterConstants.TOP_RPS_PER_LATERAL_METER;

    double forwardComponent =
        aimData.forwardM * Constants.ShooterConstants.TOP_RPS_PER_FORWARD_METER;

    double result = distComponent + lateralComponent + forwardComponent;

    return Math.max(
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Math.min(Constants.ShooterConstants.SHOOTER_MAX_RPS, result));
  }

  private double calcBottomRpsFromTagPosition(TagAimData aimData) {
    if (aimData.distanceM < 0) return Constants.ShooterConstants.SHOOTER_BOTTOM_RPS;

    double distComponent =
        Constants.ShooterConstants.SHOOTER_BOTTOM_RPS
            + (aimData.distanceM * Constants.ShooterConstants.BOTTOM_RPS_PER_METER);

    double lateralComponent =
        Math.abs(aimData.lateralM) * Constants.ShooterConstants.BOTTOM_RPS_PER_LATERAL_METER;

    double forwardComponent =
        aimData.forwardM * Constants.ShooterConstants.BOTTOM_RPS_PER_FORWARD_METER;

    double result = distComponent + lateralComponent + forwardComponent;

    return Math.max(
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Math.min(Constants.ShooterConstants.SHOOTER_MAX_RPS, result));
  }

  private double getDashboardManualHoodTarget() {
    double manualTarget =
        SmartDashboard.getNumber("Hood/ManualTargetRot", hoodManualRot);

    return Math.max(
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Math.min(Constants.ShooterConstants.HOOD_MAX_ROT, manualTarget));
  }

  private double getDashboardManualTopRps() {
    double top =
        SmartDashboard.getNumber("Shooter/ManualTopRPS", Constants.ShooterConstants.SHOOTER_TOP_RPS);

    return Math.max(
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Math.min(Constants.ShooterConstants.SHOOTER_MAX_RPS, top));
  }

  private double getDashboardManualBottomRps() {
    double bottom =
        SmartDashboard.getNumber(
            "Shooter/ManualBottomRPS", Constants.ShooterConstants.SHOOTER_BOTTOM_RPS);

    return Math.max(
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Math.min(Constants.ShooterConstants.SHOOTER_MAX_RPS, bottom));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    hoodIO.updateInputs(hoodInputs);

    shooterManualMode = SmartDashboard.getBoolean("Shooter/ManualSpeedMode", shooterManualMode);
    shooterManualTopRps = getDashboardManualTopRps();
    shooterManualBottomRps = getDashboardManualBottomRps();

    boolean hoodDashboardManual =
        SmartDashboard.getBoolean("Hood/ManualTargetFromDashboard", false);

    TagAimData aimData = getTagAimData();
    double dist = aimData.distanceM;

    switch (wanted) {
      case IDLE -> {
        targetTopRps = 0.0;
        targetBottomRps = 0.0;
        targetHoodRot = 0.0;
        targetFeedRps = 0.0;
        feedEnabled = false;
      }

      case PREPARE_SHOT -> {
        targetHoodRot = calcHoodRotFromDistance(dist);

        double top = calcTopRpsFromTagPosition(aimData);
        double bot = calcBottomRpsFromTagPosition(aimData);

        if (shooterManualMode) {
          top = shooterManualTopRps;
          bot = shooterManualBottomRps;
        } else if (Constants.ShooterConstants.LIVE_TUNING) {
          top = SmartDashboard.getNumber("Shooter/TopRPS", top);
          bot = SmartDashboard.getNumber("Shooter/BottomRPS", bot);
        }

        if (hoodDashboardManual) {
          targetHoodRot = getDashboardManualHoodTarget();
        }

        targetTopRps = top;
        targetBottomRps = bot;
        targetFeedRps = 0.0;
        feedEnabled = false;
      }

      case SHOOTING -> {
        targetHoodRot = calcHoodRotFromDistance(dist);

        double top = calcTopRpsFromTagPosition(aimData);
        double bot = calcBottomRpsFromTagPosition(aimData);
        double feed = Constants.ShooterConstants.FEEDER_RPS;

        if (shooterManualMode) {
          top = shooterManualTopRps;
          bot = shooterManualBottomRps;
        } else if (Constants.ShooterConstants.LIVE_TUNING) {
          top = SmartDashboard.getNumber("Shooter/TopRPS", top);
          bot = SmartDashboard.getNumber("Shooter/BottomRPS", bot);
        }

        if (Constants.ShooterConstants.LIVE_TUNING) {
          feed = SmartDashboard.getNumber("Shooter/FeederRPS", feed);
        }

        if (hoodDashboardManual) {
          targetHoodRot = getDashboardManualHoodTarget();
        }

        targetTopRps = top;
        targetBottomRps = bot;
        targetFeedRps = feedEnabled ? feed : 0.0;
      }
    }

    if (wanted == WantedState.IDLE) {
      shooterIO.stop();
    } else {
      shooterIO.setFlywheelVelocityRps(targetTopRps, targetBottomRps, targetFeedRps);
    }

    hoodIO.setHoodPositionRot(hoodManualMode ? hoodManualRot : targetHoodRot);

    SmartDashboard.putNumber("Vision/TagDistanceM", dist);
    SmartDashboard.putNumber("Vision/TagForwardM", aimData.forwardM);
    SmartDashboard.putNumber("Vision/TagLateralM", aimData.lateralM);

    SmartDashboard.putNumber("Shooter/TargetTopRPS", targetTopRps);
    SmartDashboard.putNumber("Shooter/TargetBottomRPS", targetBottomRps);
    SmartDashboard.putNumber("Shooter/ActualTopRPS", shooterInputs.topVelocityRps);
    SmartDashboard.putNumber("Shooter/ActualBottomRPS", shooterInputs.bottomVelocityRps);
    SmartDashboard.putBoolean("Shooter/AtSpeed", flywheelsAtSpeed());
    SmartDashboard.putBoolean("Shooter/ManualSpeedModeActive", shooterManualMode);

    SmartDashboard.putNumber("Hood/PositionDeg", hoodInputs.hoodPositionRot * 360.0);
    SmartDashboard.putNumber("Hood/TargetDeg", targetHoodRot * 360.0);
    SmartDashboard.putNumber("Hood/DistanceM", dist);
    SmartDashboard.putNumber("Hood/TargetRot", targetHoodRot);
    SmartDashboard.putNumber("Hood/ActualRot", hoodInputs.hoodPositionRot);
    SmartDashboard.putBoolean("Hood/AtTarget", hoodAtTarget());
    SmartDashboard.putNumber("Hood/PositionRot", hoodInputs.hoodPositionRot);
    SmartDashboard.putNumber("Hood/ManualRot", hoodManualRot);
    SmartDashboard.putNumber("Hood/ErrorRot", targetHoodRot - hoodInputs.hoodPositionRot);
  }

  @Override
  public void simulationPeriodic() {
    shooterIO.simulationPeriodic();
    hoodIO.simulationPeriodic();
  }
}