package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

public class ShootOnMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final Vision vision;
  private final IndexerSubsystem indexerSubsystem;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier manualOmegaSupplier;

  private final double maxSpeedMps;
  private final double maxAngularRateRps;

  private static final double kTransDeadband = 0.10;
  private static final double kRotDeadband = 0.10;
  private static final double kManualOverride = 0.12;

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
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier manualOmegaSupplier,
      double maxSpeedMps,
      double maxAngularRateRps) {

    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.vision = vision;
    this.indexerSubsystem = indexerSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.manualOmegaSupplier = manualOmegaSupplier;
    this.maxSpeedMps = maxSpeedMps;
    this.maxAngularRateRps = maxAngularRateRps;

    this.requirements = Set.of(drivetrain, shooter, indexerSubsystem);
  }

  @Override
  public void initialize() {
    shooter.setWantedState(ShooterSubsystem.WantedState.SHOOTING);

    // start: no feeding
    shooter.setFeedEnabled(false);
    indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
  }

  @Override
  public void execute() {
    double vxCmd = MathUtil.applyDeadband(xSupplier.getAsDouble(), kTransDeadband) * maxSpeedMps;
    double vyCmd = MathUtil.applyDeadband(ySupplier.getAsDouble(), kTransDeadband) * maxSpeedMps;

    double manual = MathUtil.applyDeadband(manualOmegaSupplier.getAsDouble(), kRotDeadband);
    double omegaCmdRadPerSec;

    if (Math.abs(manual) > kManualOverride) {
      omegaCmdRadPerSec = manual * maxAngularRateRps;
    } else {
      boolean hasTarget = vision.hasTarget();
      omegaCmdRadPerSec = hasTarget ? vision.calcAimOmegaRadPerSec() : 0.0;
    }

    omegaCmdRadPerSec = MathUtil.clamp(omegaCmdRadPerSec, -maxAngularRateRps, maxAngularRateRps);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(vxCmd)
            .withVelocityY(vyCmd)
            .withRotationalRate(omegaCmdRadPerSec));

    boolean ready = shooter.flywheelsAtSpeed();

   
    shooter.setFeedEnabled(ready);
    indexerSubsystem.setWantedState(
        ready ? IndexerSubsystem.WantedState.INDEX : IndexerSubsystem.WantedState.IDLE
    );
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFeedEnabled(false);
    shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
    indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
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