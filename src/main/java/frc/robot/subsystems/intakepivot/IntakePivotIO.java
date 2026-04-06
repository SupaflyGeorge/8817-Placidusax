package frc.robot.subsystems.intakepivot;

import edu.wpi.first.epilogue.Logged;

/**
 * Hardware abstraction for the intake pivot arm.
 * The pivot swings the intake down to the ground (deploy) or
 * back up into the robot frame (stow).
 */
public interface IntakePivotIO {

  @Logged
  class IntakePivotIOInputs {
    public double motorPercent;       // commanded output
    public double motorVolts;
    public double motorAmps;
    public double motorTempC;
    public double pivotPositionRot;   // continuous position from TalonFX (mechanism rotations)
    public double pivotVelocityRps;   // mechanism rotations per second
    public double pivotAbsPositionRot; // CANcoder absolute position (for debug)
    public boolean homed;             // true once we've zeroed the position
  }

  void updateInputs(IntakePivotIOInputs inputs);
  void setPercent(double percent);
  void stop();
  default void simulationPeriodic() {}
}
