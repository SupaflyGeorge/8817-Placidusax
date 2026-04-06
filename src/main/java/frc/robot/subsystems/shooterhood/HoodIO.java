package frc.robot.subsystems.shooterhood;

import edu.wpi.first.epilogue.Logged;

/**
 * Hardware abstraction for the adjustable shooter hood.
 * The hood changes the launch angle - position controlled via PID.
 */
public interface HoodIO {

  @Logged
  class HoodIOInputs {
    public double hoodPositionRot;    // mechanism rotations (from Talon internal sensor)
    public double hoodAbsPositionRot; // CANcoder absolute reading
    public double hoodTargetRot;      // what we last commanded
    public double motorVolts;
    public double motorAmps;
    public double motorTempC;
    public boolean homed;             // true after first absolute position seed
  }

  void updateInputs(HoodIOInputs inputs);

  /** Command hood to a target position in mechanism rotations. */
  void setHoodPositionRot(double hoodRot);

  void stop();
  default void simulationPeriodic() {}
}