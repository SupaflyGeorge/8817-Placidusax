package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;

public interface IndexerIO {

  @Logged
  class IndexerIOInputs {
    public double beltPercent;
    public double beltVolts;
    public double beltAmps;
    public double beltTempC;
  }

  void updateInputs(IndexerIOInputs inputs);

  void setBeltPercent(double percent);

  void stop();

  default void simulationPeriodic() {}
}