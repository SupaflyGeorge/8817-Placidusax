package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

@Logged
public class IndexerSubsystem extends SubsystemBase {

  public enum WantedState {
    IDLE,
    INDEX
  }

  private final IndexerIO io = new IndexerIOTalonFX();
  private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

  private WantedState wanted = WantedState.IDLE;

  public void setWantedState(WantedState state) {
    wanted = state;
  }

  public WantedState getWantedState() {
    return wanted;
  }
  
  public void runPercent(double pct) {
  io.setBeltPercent(pct);
 }

  public void stop() {
  io.setBeltPercent(0.0);
 }


  

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (wanted) {
      case IDLE -> io.stop();
      case INDEX -> io.setBeltPercent(Constants.IndexerConstants.BELT_PERCENT);
    }

    
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }
}