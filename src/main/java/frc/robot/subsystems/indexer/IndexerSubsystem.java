package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Belt indexer subsystem - pushes game pieces from the intake up into the shooter.
 * Simple two-state machine: IDLE (stopped) or INDEX (belt running).
 */
@Logged
public class IndexerSubsystem extends SubsystemBase {

  public enum WantedState { IDLE, INDEX, SPIT }

  private final IndexerIO io = new IndexerIOTalonFX();
  private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();
  private WantedState wanted = WantedState.IDLE;

  public void setWantedState(WantedState state) { wanted = state; }
  public WantedState getWantedState() { return wanted; }

  /** Direct speed control for commands that need custom belt speeds. */
  public void runPercent(double pct) { io.setBeltPercent(pct); }
  public void stop() { io.setBeltPercent(0.0); }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    switch (wanted) {
      case IDLE  -> io.stop();
      case INDEX -> io.setBeltPercent(Constants.IndexerConstants.BELT_PERCENT * -1);
      case SPIT  -> io.setBeltPercent(Constants.IndexerConstants.BELT_PERCENT);
    }
  }

  @Override
  public void simulationPeriodic() { io.simulationPeriodic(); }
}
