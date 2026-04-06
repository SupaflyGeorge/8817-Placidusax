package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;

/**
 * Hardware abstraction for the dual-flywheel shooter + feeder wheel.
 * Supports both velocity PID control and raw voltage output.
 */
public interface ShooterIO {

  @Logged
  class ShooterIOInputs {
    public double topVelocityRps;     // actual top flywheel speed
    public double bottomVelocityRps;  // actual bottom flywheel speed
    public double feederVelocityRps;  // actual feeder speed

    public double topAppliedVolts;
    public double bottomAppliedVolts;
    public double feederAppliedVolts;

    public double topCurrentAmps;
    public double bottomCurrentAmps;
    public double feederCurrentAmps;

    public double topTempC;
    public double bottomTempC;
    public double feederTempC;
  }

  void updateInputs(ShooterIOInputs inputs);

  /** Command all three motors to target velocities using on-motor PID. */
  void setFlywheelVelocityRps(double topRps, double bottomRps, double feedRps);

  /** Raw voltage control (used for SysId or manual testing). */
  void setFlywheelVoltage(double topVolts, double bottomVolts, double feedVolts);

  void stop();
  default void simulationPeriodic() {}
}