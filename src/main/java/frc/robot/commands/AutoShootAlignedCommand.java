package frc.robot.commands;

import java.util.Set;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

/**
 * Auto-only command: stops the drivetrain (no translation), auto-aims at the
 * hub using vision yaw, and shoots once the flywheels + hood are ready.
 *
 * Also oscillates the intake pivot up/down after a delay to agitate pieces
 * and help them feed into the shooter consistently.
 */
public class AutoShootAlignedCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final Vision vision;
  private final IntakePivotSubsystem intakePivotSubsystem;
  private final double maxAngularRateRps;

  // Pivot oscillation parameters (agitates pieces toward the feeder)
  private static final double kPivotLowRot = 0.25;
  private static final double kPivotHighRot = 0.55;
  private static final double kPivotOpenLoopPercent = 0.20;
  private static final double kPivotStartDelaySec = 1.0;  // wait before starting oscillation

  private boolean pivotMovingUp = true;
  private final Timer pivotTimer = new Timer();

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDeadband(0.0).withRotationalDeadband(0.0);

  private final Set<Subsystem> requirements;

  public AutoShootAlignedCommand(
      CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
      IndexerSubsystem indexer, Vision vision,
      IntakePivotSubsystem intakePivotSubsystem, double maxAngularRateRps) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;
    this.intakePivotSubsystem = intakePivotSubsystem;
    this.maxAngularRateRps = maxAngularRateRps;
    this.requirements = Set.of(drivetrain, shooter, indexer, intakePivotSubsystem);
  }

  @Override
  public void initialize() {
    // Start spinning flywheels but don't feed yet
    shooter.setWantedState(ShooterSubsystem.WantedState.SHOOTING);
    shooter.setFeedEnabled(false);
    indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
    intakePivotSubsystem.setWantedState(IntakePivotSubsystem.WantedState.IDLE);

    // Determine initial oscillation direction based on current position
    double pivotRot = intakePivotSubsystem.getPivotPositionRot();
    pivotMovingUp = pivotRot < (kPivotLowRot + kPivotHighRot) * 0.5;
    pivotTimer.reset();
    pivotTimer.start();
  }

  @Override
  public void execute() {
    // --- Auto-aim rotation ---
    double omega = 0.0;
    if (vision.hasTarget()) {
      omega = vision.calcAimOmegaRadPerSec();
      omega = MathUtil.clamp(omega, -maxAngularRateRps, maxAngularRateRps);
    }
    // Hold position (0 translation) while rotating to aim
    drivetrain.setControl(driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(omega));

    // --- Feed when ready ---
    boolean ready = shooter.flywheelsAtSpeed() && shooter.hoodAtTarget();
    shooter.setFeedEnabled(ready);
    indexer.setWantedState(ready ? IndexerSubsystem.WantedState.INDEX : IndexerSubsystem.WantedState.IDLE);

    // --- Pivot oscillation (agitate pieces after a delay) ---
    if (pivotTimer.hasElapsed(kPivotStartDelaySec)) {
      double pivotRot = intakePivotSubsystem.getPivotPositionRot();
      if (pivotRot >= kPivotHighRot) pivotMovingUp = false;
      else if (pivotRot <= kPivotLowRot) pivotMovingUp = true;
      intakePivotSubsystem.setOpenLoopPercent(pivotMovingUp ? kPivotOpenLoopPercent : -kPivotOpenLoopPercent);
    } else {
      intakePivotSubsystem.stopOpenLoop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivotTimer.stop();
    shooter.setFeedEnabled(false);
    shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
    indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
    intakePivotSubsystem.stopOpenLoop();
    intakePivotSubsystem.setWantedState(IntakePivotSubsystem.WantedState.IDLE);
    drivetrain.setControl(driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
  }

  @Override public boolean isFinished() { return false; }
  @Override public Set<Subsystem> getRequirements() { return requirements; }
}
