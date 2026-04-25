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

public final class Constants {
  private Constants() {}

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "arducam";

    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final double ROBOT_LENGTH_IN = 30.0;
    public static final double ROBOT_WIDTH_IN = 25.0;

    // Camera position relative to ROBOT CENTER
    // +X = forward, +Y = left, +Z = up
    public static final double CAM_X_IN = -5.0;
    public static final double CAM_Y_IN = -6.0;
    public static final double CAM_Z_IN = 21.0;

    public static final double CAM_X_METERS = Units.inchesToMeters(CAM_X_IN);
    public static final double CAM_Y_METERS = Units.inchesToMeters(CAM_Y_IN);
    public static final double CAM_Z_METERS = Units.inchesToMeters(CAM_Z_IN);

    public static final double CAM_ROLL_DEG = 0.0;
    public static final double CAM_PITCH_DEG = 20.0;
    public static final double CAM_YAW_DEG = 0.0;

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            new Translation3d(
                CAM_X_METERS,
                CAM_Y_METERS,
                CAM_Z_METERS),
            new Rotation3d(
                Units.degreesToRadians(CAM_ROLL_DEG),
                Units.degreesToRadians(CAM_PITCH_DEG),
                Units.degreesToRadians(CAM_YAW_DEG)));

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);

    public static final double MAX_AMBIGUITY = 0.25;
    public static final double MAX_DISTANCE_METERS = 4.5;
  }

  public static final class ShooterConstants {
    private ShooterConstants() {}

    public static final int TOP_ID = 9;
    public static final int MID_ID = 16;
    public static final int BOTTOM_ID = 10;
    public static final int FEEDER_ID = 11;
    public static final int HOOD_MOTOR_ID = 12;
    public static final int HOOD_CANCODER_ID = 36;
    public static final String CANBUS = "";

    public static final double FLYWHEEL_GEAR_RATIO = 1.0;

    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.15;
    public static final double kS = 0.0;
    public static final double kA = 0.01;

    public static final double SHOOTER_TOP_RPS = 28.50;
    public static final double SHOOTER_BOTTOM_RPS = 28.50;
    public static final double FEEDER_IDLE_RPS = 0.0;
    public static final double FEEDER_RPS = 60.0;
    public static final double INDEXER_RPS = 80.0;
    public static final double INDEXER_IDLE_RPS = 0.0;

    public static final double SHOOTER_MIN_RPS = 0.0;
    public static final double SHOOTER_MAX_RPS = 120.0;

    public static final double READY_TOL_RPS = 4.5;
    public static final double READY_MIN_RPS = 10.0;

    public static final double SUPPLY_LIMIT_A = 40.0;
    public static final double FEEDER_SUPPLY_LIMIT_A = 25.0;

    public static final boolean TOP_INVERTED = false;
    public static final boolean MID_INVERTED = false;
    public static final boolean BOTTOM_INVERTED = true;
    public static final boolean FEEDER_INVERTED = false;

    public static final boolean LIVE_TUNING = true;

    public static final double HOOD_GEAR_RATIO = 6.68;
    public static final double HOOD_kP = 10.0;
    public static final double HOOD_kI = 0.0;
    public static final double HOOD_kD = 0.0;
    public static final double HOOD_kV = 0.0;

    public static final double HOOD_MIN_ROT = 0.0;
    public static final double HOOD_MAX_ROT = 0.84;
    public static final double HOOD_READY_TOL_ROT = 0.01;

    public static final double[] SHOT_DISTANCE_M = {
      0.976, 1.368, 1.741, 2.067, 2.431, 2.506, 3.080, 3.410, 3.682, 3.900
    };

    public static final double[] SHOT_HOOD_ROT = {
      0.308258, 0.322734, 0.352314, 0.3586, 0.403105, 0.462666,
      0.482939, 0.592246, 0.643594, 0.692881
    };

    public static final double[] SHOT_RPS = {
      26.00, 27.0, 28.0, 27.25, 28.0, 29.00,
      31.75, 34.75, 42.25, 44.0
    };
  }

  public static final class IntakeConstants {
    private IntakeConstants() {}
    public static final int ROLLER_ID = 13;
    public static final double INTAKE_PERCENT = -0.8; // negative = pulls inward
    public static final boolean ROLLER_INVERTED = false;
    public static final double ROLLER_SUPPLY_LIMIT_A = 30.0;
  }

  public static final class IndexerConstants {
    private IndexerConstants() {}
    public static final int BELTER_ID = 15;
    public static final double BELT_PERCENT = 0.6;
    public static final boolean BELT_INVERTED = true;
    public static final double BELT_SUPPLY_LIMIT_A = 40.0;
  }

  public static final class IntakePivotConstants {
    private IntakePivotConstants() {}
    public static final int PIVOT_ID = 14;
    public static final int PIVOT_CANCODER_ID = 37;
    public static final double PIVOT_GEAR_RATIO = 36.36 / 1.0;
    public static final double MIN_ROT = 0.001;
    public static final double MAX_ROT = -0.41;
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