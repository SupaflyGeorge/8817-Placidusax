package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;

/** Hardware abstraction for the intake roller motor. */
public interface IntakeIO {

  @Logged
  class IntakeIOInputs {
    public double rollerPercent;
    public double rollerVolts;
    public double rollerAmps;
    public double rollerTempC;
  }

  void updateInputs(IntakeIOInputs inputs);
  void setRollerPercent(double percent);
  void stop();
  default void simulationPeriodic() {}
}
