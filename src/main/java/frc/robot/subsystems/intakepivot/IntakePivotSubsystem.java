package frc.robot.subsystems.intakepivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake pivot arm subsystem. Runs open-loop (duty cycle) with a
 * deceleration ramp near the travel limits so it doesn't slam to a stop.
 *
 * States:
 *   IDLE   - motor stopped, brake mode holds position
 *   DEPLOY - swing arm down to the ground
 *   STOW   - swing arm back up into the robot
 */
@Logged
public class IntakePivotSubsystem extends SubsystemBase {

  public enum WantedState { DEPLOY, STOW, IDLE }

  private final IntakePivotIO io = new IntakePivotIOTalonFX();
  private final IntakePivotIO.IntakePivotIOInputs inputs = new IntakePivotIO.IntakePivotIOInputs();
  private WantedState wanted = WantedState.IDLE;

  private static final double DEPLOY_PCT = +0.700;        // full speed deploying
  private static final double STOW_PCT = -0.700;          // full speed stowing
  private static final double MAX_ROT_BOTH_SIDES = 0.6200; // deploy limit
  private static final double SLOWDOWN_ZONE_ROT = 0.15;   // start braking this far from limit
  private static final double MIN_SLOW_PCT = 0.10;        // minimum creep speed in the zone

  public void setWantedState(WantedState state) { wanted = state; }
  public WantedState getWantedState() { return wanted; }
  public double getPivotPositionRot() { return inputs.pivotPositionRot; }

  /** Used by ShootOnMoveCommand for small oscillation movements. */
  public void setOpenLoopPercent(double percent) {
    io.setPercent(MathUtil.clamp(percent, -0.2, 0.2));
  }
  public void stopOpenLoop() { io.stop(); }

  private boolean atOrPastMax() { return Math.abs(inputs.pivotPositionRot) >= MAX_ROT_BOTH_SIDES; }
  private boolean atOrPastMin() { return inputs.pivotPositionRot <= Constants.IntakePivotConstants.MIN_ROT; }

  /**
   * Returns 0-1 scale factor: 1.0 when far from limit, ramps toward 0 near limit.
   * This gives us a smooth slowdown instead of an abrupt stop.
   */
  private double calcDecelFactor(double currentRot, double targetRot) {
    double remaining = Math.abs(targetRot - currentRot);
    if (remaining >= SLOWDOWN_ZONE_ROT) return 1.0;
    return remaining / SLOWDOWN_ZONE_ROT;
  }

  /** Apply decel factor to full speed, but never go below the creep minimum. */
  private double applyDecel(double fullPct, double currentRot, double targetRot) {
    double factor = calcDecelFactor(currentRot, targetRot);
    double scaled = fullPct * factor;
    double minOutput = Math.copySign(MIN_SLOW_PCT, fullPct);
    if (Math.abs(scaled) < MIN_SLOW_PCT) return minOutput;
    return scaled;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double out = 0.0;

    switch (wanted) {
      case IDLE -> out = 0.0;
      case DEPLOY -> {
        if (atOrPastMax()) out = 0.0;
        else out = applyDecel(DEPLOY_PCT, inputs.pivotPositionRot, MAX_ROT_BOTH_SIDES);
      }
      case STOW -> {
        if (atOrPastMin()) out = 0.0;
        else out = applyDecel(STOW_PCT, inputs.pivotPositionRot, Constants.IntakePivotConstants.MIN_ROT);
      }
    }

    if (out == 0.0) io.stop();
    else io.setPercent(out);

    // Publish debug data to SmartDashboard
    SmartDashboard.putNumber("IntakePivot/PosRot", inputs.pivotPositionRot);
    SmartDashboard.putNumber("IntakePivot/AbsRot", inputs.pivotAbsPositionRot);
    SmartDashboard.putNumber("IntakePivot/VelRps", inputs.pivotVelocityRps);
    SmartDashboard.putString("IntakePivot/Wanted", wanted.name());
    SmartDashboard.putBoolean("IntakePivot/AtMin", atOrPastMin());
    SmartDashboard.putBoolean("IntakePivot/AtMax", atOrPastMax());
    SmartDashboard.putBoolean("IntakePivot/Homed", inputs.homed);
    SmartDashboard.putNumber("IntakePivot/OutputPct", out);
  }

  @Override
  public void simulationPeriodic() { io.simulationPeriodic(); }
}
