package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake roller subsystem. Three states:
 *   IDLE   - rollers stopped
 *   INTAKE - rollers spin inward to grab game pieces
 *   SPIT   - rollers reverse to eject pieces (for clearing jams)
 */
@Logged
public class IntakeSubsystem extends SubsystemBase {

  public enum WantedState { IDLE, INTAKE, SPIT }

  private final IntakeIO io = new IntakeIOTalonFX();
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  private WantedState wanted = WantedState.IDLE;

  public void setWantedState(WantedState state) { wanted = state; }
  public WantedState getWantedState() { return wanted; }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    switch (wanted) {
      case IDLE   -> io.stop();
      case INTAKE -> io.setRollerPercent(Constants.IntakeConstants.INTAKE_PERCENT);
      case SPIT   -> io.setRollerPercent(Constants.IntakeConstants.INTAKE_PERCENT * -1.1);
    }
  }

  @Override
  public void simulationPeriodic() { io.simulationPeriodic(); }
}
