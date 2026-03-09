package frc.robot.subsystems.shooterhood;

import edu.wpi.first.epilogue.Logged;

public interface HoodIO {

  @Logged
  class HoodIOInputs {
    public double hoodPositionRot;     // mechanism rotations
    public double hoodAbsPositionRot;  // CANcoder absolute (mechanism rotations)
    public double hoodTargetRot;

    public double motorVolts;
    public double motorAmps;
    public double motorTempC;
    public boolean homed;
  }

  void updateInputs(HoodIOInputs inputs);

  void setHoodPositionRot(double hoodRot);

  void stop();

  default void simulationPeriodic() {}
}