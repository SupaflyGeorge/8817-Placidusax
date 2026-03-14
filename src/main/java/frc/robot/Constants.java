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

    public static final double ROBOT_LENGTH_IN = 30.0; // front-to-back
    public static final double ROBOT_WIDTH_IN  = 25.0; // left-to-right

    public static final double CAM_FROM_FRONT_IN = 12.25;
    public static final double CAM_FROM_LEFT_IN  = 15.75;
    public static final double CAM_HEIGHT_IN     = 19.5;

    public static final double CAM_ROLL_DEG  = 0.0;
    public static final double CAM_PITCH_DEG = 25.0;
    public static final double CAM_YAW_DEG   = 0.0;

    // WPILib robot frame: +X forward, +Y left, +Z up
    private static final double camX_in = (ROBOT_LENGTH_IN / 2.0) - CAM_FROM_FRONT_IN; // +forward
    private static final double camY_in = (ROBOT_WIDTH_IN  / 2.0) - CAM_FROM_LEFT_IN;  // +left
    private static final double camZ_in = CAM_HEIGHT_IN;                                // +up

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(camX_in),
                Units.inchesToMeters(camY_in),
                Units.inchesToMeters(camZ_in)
            ),
            new Rotation3d(
                Units.degreesToRadians(CAM_ROLL_DEG),
                Units.degreesToRadians(CAM_PITCH_DEG),
                Units.degreesToRadians(CAM_YAW_DEG)
            )
        );

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1.0);

    public static final double MAX_AMBIGUITY = 0.25;
    public static final double MAX_DISTANCE_METERS = 4.5;
  }


  public static final class ShooterConstants {
    private ShooterConstants() {}

    // CAN IDs
    public static final int TOP_ID = 9;
    public static final int BOTTOM_ID = 10;
    public static final int FEEDER_ID = 11;

    public static final int HOOD_MOTOR_ID = 12;
    public static final int HOOD_CANCODER_ID = 36;

    // No CANivore (RIO bus)
    public static final String CANBUS = ""; 

    // Flywheels
    public static final double FLYWHEEL_GEAR_RATIO = 1.0; //orRot / wheelRot

  
    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.12;
    public static final double kS = 0.0;
    public static final double kA = 0.0;

    // Targets 
    public static final double SHOOTER_TOP_RPS = 30.50;
    public static final double SHOOTER_BOTTOM_RPS = 30.50;
    public static final double FEEDER_IDLE_RPS = 0.0;
    public static final double FEEDER_RPS = 100.0;
    public static final double INDEXER_RPS = 80.0;
    public static final double INDEXER_IDLE_RPS = 0.0;

    public static final double SHOOTER_MIN_RPS = 0.0;
    public static final double SHOOTER_MAX_RPS = 120.0;

    public static final double TOP_RPS_PER_METER = 4.0;
    public static final double BOTTOM_RPS_PER_METER = 4.0;

    public static final double TOP_RPS_PER_LATERAL_METER = 2.0;
    public static final double BOTTOM_RPS_PER_LATERAL_METER = 2.0;

    public static final double TOP_RPS_PER_FORWARD_METER = 0.0;
    public static final double BOTTOM_RPS_PER_FORWARD_METER = 0.0;

   

    // Ready checks
    public static final double READY_TOL_RPS = 3.0;
    public static final double READY_MIN_RPS = 10.0;

    // Current limits
    public static final double SUPPLY_LIMIT_A = 60.0;
    public static final double FEEDER_SUPPLY_LIMIT_A = 60.0;

    // Inverts
    public static final boolean TOP_INVERTED = false;
    public static final boolean BOTTOM_INVERTED = true;
    public static final boolean FEEDER_INVERTED = false;

    // Optional live tuning
    public static final boolean LIVE_TUNING = true;

   
    public static final double HOOD_GEAR_RATIO = 6.68;

    // Hood PID (start values — tune)
    public static final double HOOD_kP = 40.0;
    public static final double HOOD_kI = 0.0;
    public static final double HOOD_kD = 0.0;
    public static final double HOOD_kV = 0.0;

    // Safety limits (mechanism rotations)
    public static final double HOOD_MIN_ROT = 0.0;    
    public static final double HOOD_MAX_ROT = 0.84;   

    // Shot model 
    // angleRot = A * distanceMeters + B, then clamped to min/max
    public static final double HOOD_A_ROT_PER_M = 0.05;
    public static final double HOOD_B_ROT = 0.08;

    // Feeder only allowed when hood is close enough
    public static final double HOOD_READY_TOL_ROT = 0.01;

    // hood quadratic variables
    public static final double HOOD_QUADRATIC_A = -0.0452125;
    public static final double HOOD_QUADRATIC_B = 0.357654;
    public static final double HOOD_QUADRATIC_C = -0.284559 ;
  
  }


public static final class IntakeConstants {
  private IntakeConstants() {}

  // CAN IDsj
  public static final int ROLLER_ID = 13;

  // Output
  public static final double INTAKE_PERCENT = 0.50;

  // Invert
  public static final boolean ROLLER_INVERTED = false;

  // Current limit (safe)
  public static final double ROLLER_SUPPLY_LIMIT_A = 60.0;

  
}

public static final class IndexerConstants {
  private IndexerConstants() {}

  // CAN IDs
  public static final int BELTER_ID = 15;

  // Output
  public static final double BELT_PERCENT = 0.8;

  // Invert
  public static final boolean BELT_INVERTED = true;

  // Current limit (safe)
  public static final double BELT_SUPPLY_LIMIT_A = 60.0;
}



public static final class IntakePivotConstants {
  private IntakePivotConstants() {}

  // CAN IDs (
  public static final int PIVOT_ID = 14;

  // Throughbore Encoder
  public static final int PIVOT_CANCODER_ID = 37;

  // Gear ratio: motorRot / pivotRot (CHANGE to your real ratio)
  public static final double PIVOT_GEAR_RATIO = 36.36/1;


  // Safety clamp (mechanism rotations)
  public static final double MIN_ROT = 0.05;
  public static final double MAX_ROT = 0.55;

  // Position loop gains (starter)
  public static final double kP = 60.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0;

  // Invert
  public static final boolean PIVOT_INVERTED = true;

  // Current limit
  public static final double PIVOT_SUPPLY_LIMIT_A = 60.0;

  // “Ready” tolerance
  public static final double READY_TOL_ROT = 0.01;

  // Offset
  public static final double CANCODER_OFFSET_ROT = 0.0;
}
}
