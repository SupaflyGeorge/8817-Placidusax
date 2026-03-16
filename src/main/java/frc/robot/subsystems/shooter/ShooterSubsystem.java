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

    public TagAimData(double d, double f, double l) {
      distanceM = d;
      forwardM = f;
      lateralM = l;
    }
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

  private boolean hoodManualMode = false;
  private double hoodManualRot = 0.0;

  private boolean shooterManualMode = false;
  private double shooterManualTopRps = 0.0;
  private double shooterManualBottomRps = 0.0;

  private double shooterRpsOffset = 0.0;

  private static final double READY_TOL_RPS = Constants.ShooterConstants.READY_TOL_RPS;
  private static final double READY_MIN_RPS = Constants.ShooterConstants.READY_MIN_RPS;

  public ShooterSubsystem() {

    SmartDashboard.putBoolean("Shooter/ManualSpeedMode", false);
    SmartDashboard.putNumber("Shooter/ManualTopRPS", Constants.ShooterConstants.SHOOTER_TOP_RPS);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", Constants.ShooterConstants.SHOOTER_BOTTOM_RPS);
    SmartDashboard.putNumber("Shooter/RpsOffset", 0.0);

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
    if (enable) {
      hoodManualRot = hoodInputs.hoodPositionRot;
    }
  }

  public boolean isHoodManualMode() {
    return hoodManualMode;
  }

  public void adjustHoodManual(double deltaRot) {
    hoodManualRot += deltaRot;
    hoodManualRot =
        Math.max(
            Constants.ShooterConstants.HOOD_MIN_ROT,
            Math.min(Constants.ShooterConstants.HOOD_MAX_ROT, hoodManualRot));
  }

  public void setShooterManualMode(boolean enable) {
    shooterManualMode = enable;
    SmartDashboard.putBoolean("Shooter/ManualSpeedMode", enable);
  }

  public boolean isShooterManualMode() {
    return shooterManualMode;
  }

  public void setManualShooterSpeeds(double topRps, double bottomRps) {
    shooterManualTopRps = topRps;
    shooterManualBottomRps = bottomRps;
    SmartDashboard.putNumber("Shooter/ManualTopRPS", topRps);
    SmartDashboard.putNumber("Shooter/ManualBottomRPS", bottomRps);
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
    double avg = (Math.abs(targetTopRps) + Math.abs(targetBottomRps)) * 0.5;

    if (avg < READY_MIN_RPS) return false;

    return Math.abs(shooterInputs.topVelocityRps - targetTopRps) <= READY_TOL_RPS
        && Math.abs(shooterInputs.bottomVelocityRps - targetBottomRps) <= READY_TOL_RPS;
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean hoodAtTarget() {
    return Math.abs(hoodInputs.hoodPositionRot - targetHoodRot)
        <= Constants.ShooterConstants.HOOD_READY_TOL_ROT;
  }

  @NotLogged
  private TagAimData getTagAimData() {
    PhotonPipelineResult res = camera.getLatestResult();

    if (res == null || !res.hasTargets()) {
      return new TagAimData(-1.0, -1.0, 0.0);
    }

    boolean blue =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;

    List<Integer> hubCenters =
        blue ? List.of(18, 24, 26) : List.of(2, 8, 10);

    List<Integer> hubIds =
        blue ? List.of(21, 24, 19, 20, 18, 27)
             : List.of(2, 11, 8, 5, 9, 10);

    double dist = 0.0;
    double forward = 0.0;
    double lateral = 0.0;
    int count = 0;

    for (PhotonTrackedTarget t : res.getTargets()) {
      if (hubIds.contains(t.getFiducialId()) || hubCenters.contains(t.getFiducialId())) {
        double x = t.getBestCameraToTarget().getX();
        double y = t.getBestCameraToTarget().getY();

        dist += Math.hypot(x, y);
        forward += x;
        lateral += y;
        count++;
      }
    }

    if (count == 0) {
      return new TagAimData(-1.0, -1.0, 0.0);
    }

    return new TagAimData(dist / count, forward / count, lateral / count);
  }

  @NotLogged
  public double getDistanceToTagMeters() {
    return getTagAimData().distanceM;
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

  private double calcMapHood(double dist) {
    if (dist < 0.0) return hoodInputs.hoodPositionRot;

    return Math.max(
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Math.min(
            Constants.ShooterConstants.HOOD_MAX_ROT,
            interpolate(
                dist,
                Constants.ShooterConstants.SHOT_DISTANCE_M,
                Constants.ShooterConstants.SHOT_HOOD_ROT)));
  }

  private double calcMapRps(double dist) {
    if (dist < 0.0) {
      return Constants.ShooterConstants.SHOOTER_TOP_RPS;
    }

    return Math.max(
        Constants.ShooterConstants.SHOOTER_MIN_RPS,
        Math.min(
            Constants.ShooterConstants.SHOOTER_MAX_RPS,
            interpolate(
                dist,
                Constants.ShooterConstants.SHOT_DISTANCE_M,
                Constants.ShooterConstants.SHOT_RPS)));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    hoodIO.updateInputs(hoodInputs);

    shooterManualMode =
        SmartDashboard.getBoolean("Shooter/ManualSpeedMode", shooterManualMode);

    shooterManualTopRps =
        SmartDashboard.getNumber(
            "Shooter/ManualTopRPS",
            Constants.ShooterConstants.SHOOTER_TOP_RPS);

    shooterManualBottomRps =
        SmartDashboard.getNumber(
            "Shooter/ManualBottomRPS",
            Constants.ShooterConstants.SHOOTER_BOTTOM_RPS);

    boolean hoodManualDashboard =
        SmartDashboard.getBoolean("Hood/ManualTargetFromDashboard", false);

    TagAimData aim = getTagAimData();
    double dist = aim.distanceM;

    mapHoodRot = calcMapHood(dist);
    mapRps = calcMapRps(dist);

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

        double top;
        double bot;

        if (shooterManualMode) {
          top = shooterManualTopRps;
          bot = shooterManualBottomRps;
        } else {
          top = mapRps;
          bot = mapRps;

          if (Constants.ShooterConstants.LIVE_TUNING) {
            top = SmartDashboard.getNumber("Shooter/TopRPS", top);
            bot = SmartDashboard.getNumber("Shooter/BottomRPS", bot);
          }
        }

        targetTopRps = top + shooterRpsOffset;
        targetBottomRps = bot + shooterRpsOffset;

        if (hoodManualDashboard) {
          targetHoodRot = SmartDashboard.getNumber("Hood/ManualTargetRot", targetHoodRot);
        }

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
    }

    if (wanted == WantedState.IDLE) {
      shooterIO.stop();
    } else {
      shooterIO.setFlywheelVelocityRps(targetTopRps, targetBottomRps, targetFeedRps);
    }

    hoodIO.setHoodPositionRot(hoodManualMode ? hoodManualRot : targetHoodRot);

    SmartDashboard.putNumber("Vision/TagDistanceM", dist);
    SmartDashboard.putNumber("Vision/TagForwardM", aim.forwardM);
    SmartDashboard.putNumber("Vision/TagLateralM", aim.lateralM);

    SmartDashboard.putNumber("Shooter/TargetTopRPS", targetTopRps);
    SmartDashboard.putNumber("Shooter/TargetBottomRPS", targetBottomRps);
    SmartDashboard.putBoolean("Shooter/AtSpeed", flywheelsAtSpeed());
    SmartDashboard.putNumber("Shooter/MapBaseRPS", mapRps);

    SmartDashboard.putNumber("Hood/PositionDeg", hoodInputs.hoodPositionRot * 360.0);
    SmartDashboard.putNumber("Hood/TargetDeg", targetHoodRot * 360.0);
    SmartDashboard.putBoolean("Hood/AtTarget", hoodAtTarget());
    SmartDashboard.putNumber("Hood/MapBaseRot", mapHoodRot);
    SmartDashboard.putNumber("Hood/ManualRot", hoodManualRot);
    SmartDashboard.putBoolean("Hood/ManualMode", hoodManualMode);
  }

  @Override
  public void simulationPeriodic() {
    shooterIO.simulationPeriodic();
    hoodIO.simulationPeriodic();
  }


} // George was here :PPPPP