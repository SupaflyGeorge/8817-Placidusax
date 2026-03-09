package frc.robot.subsystems.intakepivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

@Logged
public class IntakePivotSubsystem extends SubsystemBase {

  public enum WantedState {
    DEPLOY,
    STOW,
    IDLE
  }

  private final IntakePivotIO io = new IntakePivotIOTalonFX();
  private final IntakePivotIO.IntakePivotIOInputs inputs = new IntakePivotIO.IntakePivotIOInputs();

  private WantedState wanted = WantedState.IDLE;

  private static final double DEPLOY_PCT = +0.800;
  private static final double STOW_PCT = -0.800;

  public void setWantedState(WantedState state) {
    wanted = state;
  }

  public WantedState getWantedState() {
    return wanted;
  }

  // for your current sign convention:
  // MIN_ROT = 0.0
  // MAX_ROT = -0.48
  private boolean atOrPastMax() {
    return inputs.pivotPositionRot >= Constants.IntakePivotConstants.MAX_ROT;
  }

  private boolean atOrPastMin() {
    return inputs.pivotPositionRot <= Constants.IntakePivotConstants.MIN_ROT;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    double out = 0.0;

    switch (wanted) {
      case IDLE -> out = 0.0;

      case DEPLOY -> {
        if (atOrPastMax()) out = 0.0;
        else out = DEPLOY_PCT;
      }

      case STOW -> {
        if (atOrPastMin()) out = 0.0;
        else out = STOW_PCT;
      }
    }

    out = MathUtil.clamp(out, -0.2, 0.2);

    if (out == 0.0) io.stop();
    else io.setPercent(out);

    SmartDashboard.putNumber("IntakePivot/PosRot", inputs.pivotPositionRot);
    SmartDashboard.putNumber("IntakePivot/AbsRot", inputs.pivotAbsPositionRot);
    SmartDashboard.putNumber("IntakePivot/VelRps", inputs.pivotVelocityRps);
    SmartDashboard.putString("IntakePivot/Wanted", wanted.name());
    SmartDashboard.putBoolean("IntakePivot/AtMin", atOrPastMin());
    SmartDashboard.putBoolean("IntakePivot/AtMax", atOrPastMax());
    SmartDashboard.putBoolean("IntakePivot/Homed", inputs.homed);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }
}