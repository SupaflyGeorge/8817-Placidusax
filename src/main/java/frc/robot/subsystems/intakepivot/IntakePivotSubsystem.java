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

  private static final double DEPLOY_PCT = +0.200;
  private static final double STOW_PCT = -0.200;

  public void setWantedState(WantedState state) {
    wanted = state;
  }

  public WantedState getWantedState() {
    return wanted;
  }

  public double getPivotPositionRot() {
    return inputs.pivotPositionRot;
  }

  public void setOpenLoopPercent(double percent) {
    io.setPercent(MathUtil.clamp(percent, -0.2, 0.2));
  }

  public void stopOpenLoop() {
    io.stop();
  }

  // MAX is more negative on your robot
  private boolean atOrPastMax() {
    return inputs.pivotPositionRot <= Constants.IntakePivotConstants.MAX_ROT;
  }

  // MIN is more positive on your robot
  
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
        if (atOrPastMax()) {
          out = 0.0;
        } else {
          out = DEPLOY_PCT;
        }
      }

      case STOW -> {
        if (atOrPastMin()) {
          out = 0.0;
        } else {
          out = STOW_PCT;
        }
      }
    }

    if (out == 0.0) {
      io.stop();
    } else {
      io.setPercent(out);
    }

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