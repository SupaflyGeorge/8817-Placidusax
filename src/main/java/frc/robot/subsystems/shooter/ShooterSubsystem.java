package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
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

  private boolean hoodManualMode = false;
  private double hoodManualRot = 0.0;

  @SuppressWarnings("unused")
  private double filteredDistM = -1;

  private static final double READY_TOL_RPS = Constants.ShooterConstants.READY_TOL_RPS;
  private static final double READY_MIN_RPS = Constants.ShooterConstants.READY_MIN_RPS;

  public ShooterSubsystem() {
    if (Constants.ShooterConstants.LIVE_TUNING) {
      SmartDashboard.putNumber("Shooter/TopRPS", Constants.ShooterConstants.SHOOTER_TOP_RPS);
      SmartDashboard.putNumber("Shooter/BottomRPS", Constants.ShooterConstants.SHOOTER_BOTTOM_RPS);
      SmartDashboard.putNumber("Shooter/Feeder%", Constants.ShooterConstants.FEEDER_RPS);
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

  // NEW: command uses this
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
  public double getDistanceToTagMeters() {
    PhotonPipelineResult res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) return -1.0;

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

    double transTotal = 0.0;
    int tagCount = 0;

    var targets = res.getTargets();
    if (targets.size() == 1) {
      PhotonTrackedTarget t = targets.get(0);
      if (hubIDs.contains(t.getFiducialId())) {
        transTotal += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
        tagCount++;
      }
    } else if (targets.size() == 2) {
      var t0 = targets.get(0);
      var t1 = targets.get(1);
      if ((t1.getFiducialId() - t0.getFiducialId())
          == (hubCenters.contains(t0.getFiducialId()) ? 1 : -1)) {
        for (var t : targets) {
          transTotal += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
          tagCount++;
        }
      } else {
        for (var t : targets) {
          if (hubCenters.contains(t.getFiducialId())) {
            transTotal += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
            tagCount++;
          }
        }
      }
    } else if (targets.size() == 3) {
      for (var t : targets) {
        if (hubCenters.contains(t.getFiducialId())) {
          transTotal += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
          tagCount++;
        }
      }
    } else {
      for (var t : targets) {
        if (hubIDs.contains(t.getFiducialId())) {
          transTotal += Math.hypot(t.getBestCameraToTarget().getX(), t.getBestCameraToTarget().getY());
          tagCount++;
        }
      }
    }

    if (tagCount == 0) return -1.0;
    return transTotal / tagCount;
  }

  private double calcHoodRotFromDistance(double distM) {
    if (distM < 0) return 0.0;
    System.out.println("distM: " + distM);

    final double a = Constants.ShooterConstants.HOOD_QUADRATIC_A;
    final double b = Constants.ShooterConstants.HOOD_QUADRATIC_B;
    final double c = Constants.ShooterConstants.HOOD_QUADRATIC_C;

    double formula = a * Math.pow(distM, 2) + b * distM + c;

    return Math.max(
        Constants.ShooterConstants.HOOD_MIN_ROT,
        Math.min(Constants.ShooterConstants.HOOD_MAX_ROT, formula));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    hoodIO.updateInputs(hoodInputs);

    switch (wanted) {
      case IDLE -> {
        targetTopRps = 0.0;
        targetBottomRps = 0.0;
        targetHoodRot = 0.0;
        targetFeedRps = 0.0;
        feedEnabled = false;
      }

      case PREPARE_SHOT -> {
        double dist = getDistanceToTagMeters();
        targetHoodRot = calcHoodRotFromDistance(dist);

        double top = Constants.ShooterConstants.SHOOTER_TOP_RPS;
        double bot = Constants.ShooterConstants.SHOOTER_BOTTOM_RPS;

        if (Constants.ShooterConstants.LIVE_TUNING) {
          top = SmartDashboard.getNumber("Shooter/TopRPS", top);
          bot = SmartDashboard.getNumber("Shooter/BottomRPS", bot);
        }

        targetTopRps = top;
        targetBottomRps = bot;
        targetFeedRps = 0.0;
        feedEnabled = false;
      }

      case SHOOTING -> {
        double dist = getDistanceToTagMeters();
        targetHoodRot = calcHoodRotFromDistance(dist);

        double top = Constants.ShooterConstants.SHOOTER_TOP_RPS;
        double bot = Constants.ShooterConstants.SHOOTER_BOTTOM_RPS;
        double feed = Constants.ShooterConstants.FEEDER_RPS;

        if (Constants.ShooterConstants.LIVE_TUNING) {
          top = SmartDashboard.getNumber("Shooter/TopRPS", top);
          bot = SmartDashboard.getNumber("Shooter/BottomRPS", bot);
          feed = SmartDashboard.getNumber("Shooter/Feeder%", feed);
        }

        targetTopRps = top;
        targetBottomRps = bot;

        // KEY: feeder only spins when command enables it
        targetFeedRps = feedEnabled ? feed : 0.0;
      }
    }

    if (wanted == WantedState.IDLE) {
      shooterIO.stop();
    } else {
      shooterIO.setFlywheelVelocityRps(targetTopRps, targetBottomRps, targetFeedRps);
    }

    hoodIO.setHoodPositionRot(hoodManualMode ? hoodManualRot : targetHoodRot);

    double dist = getDistanceToTagMeters();
    SmartDashboard.putNumber("Vision/TagDistanceM", dist);

    SmartDashboard.putNumber("Hood/PositionDeg", hoodInputs.hoodPositionRot * 360.0);
    SmartDashboard.putNumber("Hood/TargetDeg", targetHoodRot * 360.0);

    SmartDashboard.putNumber("Hood/DistanceM", dist);
    SmartDashboard.putNumber("Hood/TargetRot", targetHoodRot);
    SmartDashboard.putNumber("Hood/ActualRot", hoodInputs.hoodPositionRot);
    SmartDashboard.putBoolean("Hood/AtTarget", hoodAtTarget());
    SmartDashboard.putNumber("Hood/PositionRot", hoodInputs.hoodPositionRot);
    SmartDashboard.putNumber("Hood/ManuelRot", hoodManualRot);
    SmartDashboard.putNumber("Hood/ErrorRot", targetHoodRot - hoodInputs.hoodPositionRot);
  }

  @Override
  public void simulationPeriodic() {
    shooterIO.simulationPeriodic();
    hoodIO.simulationPeriodic();
  }
}