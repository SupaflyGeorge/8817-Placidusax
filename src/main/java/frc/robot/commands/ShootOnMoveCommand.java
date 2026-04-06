package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

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

public class ShootOnMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final Vision vision;
  private final IndexerSubsystem indexerSubsystem;
  private final IntakePivotSubsystem intakePivotSubsystem;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier manualOmegaSupplier;

  private final double maxSpeedMps;
  private final double maxAngularRateRps;

  private static final double kTransDeadband = 0.10;
  private static final double kRotDeadband = 0.10;
  private static final double kManualOverride = 0.12;

  private static final double kPivotLowRot = 0.25;
  private static final double kPivotHighRot = 0.55;
  private static final double kPivotOpenLoopPercent = 0.20;
  private static final double kPivotStartDelaySec = 1.0;
  private static final double kPivotMidRot = (kPivotLowRot + kPivotHighRot) * 0.5;

  private static final double kJitterAmplitudeMps = 0.05;
  private static final double kJitterFrequencyHz = 7.0;
  private static final double kJitterPeriodSec = 1.0 / kJitterFrequencyHz;
  private static final double kJitterHalfPeriodSec = kJitterPeriodSec * 0.5;

  private boolean pivotMovingUp = true;
  private final Timer pivotTimer = new Timer();
  private final Timer jitterTimer = new Timer();

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(kTransDeadband)
          .withRotationalDeadband(kRotDeadband);

  private final Set<Subsystem> requirements;

  public ShootOnMoveCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      Vision vision,
      IndexerSubsystem indexerSubsystem,
      IntakePivotSubsystem intakePivotSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier manualOmegaSupplier,
      double maxSpeedMps,
      double maxAngularRateRps) {

    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.vision = vision;
    this.indexerSubsystem = indexerSubsystem;
    this.intakePivotSubsystem = intakePivotSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.manualOmegaSupplier = manualOmegaSupplier;
    this.maxSpeedMps = maxSpeedMps;
    this.maxAngularRateRps = maxAngularRateRps;

    requirements = Set.of(drivetrain, shooter, indexerSubsystem, intakePivotSubsystem);
  }

  @Override
  public void initialize() {
    shooter.setWantedState(ShooterSubsystem.WantedState.SHOOTING);
    shooter.setFeedEnabled(false);
    indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);

    intakePivotSubsystem.setWantedState(IntakePivotSubsystem.WantedState.IDLE);

    pivotMovingUp = intakePivotSubsystem.getPivotPositionRot() < kPivotMidRot;

    pivotTimer.restart();
    jitterTimer.restart();
  }

  @Override
  public void execute() {
    double xInput = MathUtil.applyDeadband(xSupplier.getAsDouble(), kTransDeadband);
    double yInput = MathUtil.applyDeadband(ySupplier.getAsDouble(), kTransDeadband);
    double manualOmegaInput =
        MathUtil.applyDeadband(manualOmegaSupplier.getAsDouble(), kRotDeadband);

    double vxCmd = xInput * maxSpeedMps;
    double vyCmd = yInput * maxSpeedMps;

    double omegaCmdRadPerSec;
    if (Math.abs(manualOmegaInput) > kManualOverride) {
      omegaCmdRadPerSec = manualOmegaInput * maxAngularRateRps;
    } else {
      omegaCmdRadPerSec = vision.hasTarget() ? vision.calcAimOmegaRadPerSec() : 0.0;
    }

    omegaCmdRadPerSec = MathUtil.clamp(omegaCmdRadPerSec, -maxAngularRateRps, maxAngularRateRps);

    boolean ready = shooter.flywheelsAtSpeed();

    if (ready) {
      double phaseSec = jitterTimer.get() % kJitterPeriodSec;
      vyCmd += phaseSec < kJitterHalfPeriodSec ? kJitterAmplitudeMps : -kJitterAmplitudeMps;
    }

    drivetrain.setControl(
        driveRequest
            .withVelocityX(vxCmd)
            .withVelocityY(vyCmd)
            .withRotationalRate(omegaCmdRadPerSec));

    shooter.setFeedEnabled(ready);
    indexerSubsystem.setWantedState(
        ready ? IndexerSubsystem.WantedState.INDEX : IndexerSubsystem.WantedState.IDLE);

    if (!pivotTimer.hasElapsed(kPivotStartDelaySec)) {
      intakePivotSubsystem.stopOpenLoop();
      return;
    }

    double pivotRot = intakePivotSubsystem.getPivotPositionRot();

    if (pivotRot >= kPivotHighRot) {
      pivotMovingUp = false;
    } else if (pivotRot <= kPivotLowRot) {
      pivotMovingUp = true;
    }

    intakePivotSubsystem.setOpenLoopPercent(
        pivotMovingUp ? kPivotOpenLoopPercent : -kPivotOpenLoopPercent);
  }

  @Override
  public void end(boolean interrupted) {
    pivotTimer.stop();
    jitterTimer.stop();

    shooter.setFeedEnabled(false);
    shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
    indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);

    intakePivotSubsystem.stopOpenLoop();
    intakePivotSubsystem.setWantedState(IntakePivotSubsystem.WantedState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}