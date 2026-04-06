package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;

/**
 * Hardware abstraction for the belt indexer.
 * Defines WHAT the indexer can do without caring HOW (real motor vs simulation).
 */
public interface IndexerIO {

  /** Sensor data read every loop cycle. */
  @Logged
  class IndexerIOInputs {
    public double beltPercent;  // commanded duty cycle
    public double beltVolts;    // measured voltage
    public double beltAmps;     // current draw
    public double beltTempC;    // motor temp
  }

  void updateInputs(IndexerIOInputs inputs);
  void setBeltPercent(double percent);
  void stop();
  default void simulationPeriodic() {}
}
