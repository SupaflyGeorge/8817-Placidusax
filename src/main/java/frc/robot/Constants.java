package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Central place for every magic number on the robot.
 * Nothing here changes at runtime. Each mechanism gets its own inner class
 * so you can jump straight to what you need.
 */
public final class Constants {
  private Constants() {}

  /** Camera + AprilTag pose estimation settings. */
  public static final class VisionConstants {
    public static final String CAMERA_NAME = "arducam";

    // Official 2026 field tag positions
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Robot dimensions (inches) used to compute camera offset from center
    public static final double ROBOT_LENGTH_IN = 30.0;
    public static final double ROBOT_WIDTH_IN = 25.0;

    // Camera mount position measured from front edge, left edge, and ground
    public static final double CAM_FROM_FRONT_IN = 12.25;
    public static final double CAM_FROM_LEFT_IN = 15.75;
    public static final double CAM_HEIGHT_IN = 19.5;

    // Camera tilt angles
    public static final double CAM_ROLL_DEG = 0.0;
    public static final double CAM_PITCH_DEG = 25.0;  // tilted up 25 degrees
    public static final double CAM_YAW_DEG = 0.0;

    // Convert from-edge to center-relative (X=forward, Y=left, Z=up)
    private static final double camX_in = (ROBOT_LENGTH_IN / 2.0) - CAM_FROM_FRONT_IN;
    private static final double camY_in = (ROBOT_WIDTH_IN / 2.0) - CAM_FROM_LEFT_IN;
    private static final double camZ_in = CAM_HEIGHT_IN;

    // 3D transform: "where is the camera relative to robot center?"
    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(camX_in),
                Units.inchesToMeters(camY_in),
                Units.inchesToMeters(camZ_in)),
            new Rotation3d(
                Units.degreesToRadians(CAM_ROLL_DEG),
                Units.degreesToRadians(CAM_PITCH_DEG),
                Units.degreesToRadians(CAM_YAW_DEG)));

    // Kalman filter trust: bigger = less trust. Single tags are noisier.
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);

    public static final double MAX_AMBIGUITY = 0.25;        // reject noisy detections
    public static final double MAX_DISTANCE_METERS = 4.5;   // ignore far-away tags
  }

  /** Dual-flywheel shooter, feeder wheel, and adjustable hood. */
  public static final class ShooterConstants {
    private ShooterConstants() {}

    // CAN IDs
    public static final int TOP_ID = 9;
    public static final int BOTTOM_ID = 10;
    public static final int FEEDER_ID = 11;
    public static final int HOOD_MOTOR_ID = 12;
    public static final int HOOD_CANCODER_ID = 36;
    public static final String CANBUS = "";

    public static final double FLYWHEEL_GEAR_RATIO = 1.0;  // 1:1 sensor to flywheel

    // Flywheel PID + feedforward (kV is the main one: volts per RPS)
    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.12;
    public static final double kS = 0.0;
    public static final double kA = 0.0;

    // Default flywheel speeds (rotations per second)
    public static final double SHOOTER_TOP_RPS = 30.50;
    public static final double SHOOTER_BOTTOM_RPS = 30.50;
    public static final double FEEDER_IDLE_RPS = 0.0;
    public static final double FEEDER_RPS = 100.0;
    public static final double INDEXER_RPS = 80.0;
    public static final double INDEXER_IDLE_RPS = 0.0;

    // Safety clamps
    public static final double SHOOTER_MIN_RPS = 0.0;
    public static final double SHOOTER_MAX_RPS = 120.0;

    // "Ready" when within this tolerance and above minimum speed
    public static final double READY_TOL_RPS = 3.0;
    public static final double READY_MIN_RPS = 10.0;

    // Current limits
    public static final double SUPPLY_LIMIT_A = 40.0;
    public static final double FEEDER_SUPPLY_LIMIT_A = 20.0;

    // Motor directions
    public static final boolean TOP_INVERTED = false;
    public static final boolean BOTTOM_INVERTED = true;
    public static final boolean FEEDER_INVERTED = false;

    // When true, feeder RPS is tunable from SmartDashboard
    public static final boolean LIVE_TUNING = true;

    // Hood position control
    public static final double HOOD_GEAR_RATIO = 6.68;
    public static final double HOOD_kP = 40.0;
    public static final double HOOD_kI = 0.0;
    public static final double HOOD_kD = 0.0;
    public static final double HOOD_kV = 0.0;
    public static final double HOOD_MIN_ROT = 0.0;    // flat
    public static final double HOOD_MAX_ROT = 0.84;   // max angle
    public static final double HOOD_READY_TOL_ROT = 0.01;

    // Shot lookup tables: measured on the real robot at each distance.
    // The code interpolates between these points.
    public static final double[] SHOT_DISTANCE_M = {
      0.976, 1.368, 1.741, 2.067, 2.431, 2.506, 3.080, 3.410, 3.682, 3.900
    };
    public static final double[] SHOT_HOOD_ROT = {
      0.14258, 0.302734, 0.372314, 0.413086, 0.433105, 0.482666, 0.512939, 0.552246, 0.683594, 0.702881
    };
    public static final double[] SHOT_RPS = {
      26.25, 27.25, 29.25, 30.0, 31.25, 31.75, 32.75, 33.0, 34, 35
    };
  }

  /** Intake roller - one motor spinning to suck in game pieces. */
  public static final class IntakeConstants {
    private IntakeConstants() {}
    public static final int ROLLER_ID = 13;
    public static final double INTAKE_PERCENT = -0.7;  // negative = pulls inward
    public static final boolean ROLLER_INVERTED = false;
    public static final double ROLLER_SUPPLY_LIMIT_A = 20.0;
  }

  /** Belt indexer - moves game pieces from intake up into the shooter. */
  public static final class IndexerConstants {
    private IndexerConstants() {}
    public static final int BELTER_ID = 15;
    public static final double BELT_PERCENT = 1.0;
    public static final boolean BELT_INVERTED = true;
    public static final double BELT_SUPPLY_LIMIT_A = 32.0;
  }

  /** Intake pivot arm - swings intake down (deploy) or up (stow). Open-loop controlled. */
  public static final class IntakePivotConstants {
    private IntakePivotConstants() {}
    public static final int PIVOT_ID = 14;
    public static final int PIVOT_CANCODER_ID = 37;
    public static final double PIVOT_GEAR_RATIO = 36.36 / 1.0; // 36:1 reduction
    public static final double MIN_ROT = 0.001;   // stowed position
    public static final double MAX_ROT = -0.41;   // fully deployed
    public static final double kP = 60.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final boolean PIVOT_INVERTED = true;
    public static final double PIVOT_SUPPLY_LIMIT_A = 40.0;
    public static final double READY_TOL_ROT = 0.01;
    public static final double CANCODER_OFFSET_ROT = 0.0;
  }
}