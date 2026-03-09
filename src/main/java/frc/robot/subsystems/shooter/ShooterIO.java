package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;

public interface ShooterIO {

  @Logged
  class ShooterIOInputs {
    public double topVelocityRps;
    public double bottomVelocityRps;
    public double feederVelocityRps;

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

  void setFlywheelVelocityRps(double topRps, double bottomRps, double feedRps);

  void setFlywheelVoltage(double topVolts, double bottomVolts, double feedVolts);


  void stop();

  default void simulationPeriodic() {}
}