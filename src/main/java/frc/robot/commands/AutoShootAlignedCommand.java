package frc.robot.commands;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoShootAlignedCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final Vision vision;

  private final double maxAngularRateRps;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0)
          .withRotationalDeadband(0.0);

  private final Set<Subsystem> requirements;

  public AutoShootAlignedCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      Vision vision,
      double maxAngularRateRps) {

    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;
    this.maxAngularRateRps = maxAngularRateRps;

    this.requirements = Set.of(drivetrain, shooter, indexer);
  }

  @Override
  public void initialize() {
    shooter.setWantedState(ShooterSubsystem.WantedState.SHOOTING);
    shooter.setFeedEnabled(false);
    indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
  }

  @Override
  public void execute() {
    double omega = 0.0;

    if (vision.hasTarget()) {
      omega = vision.calcAimOmegaRadPerSec();
      omega = MathUtil.clamp(omega, -maxAngularRateRps, maxAngularRateRps);
    }

    // stay still, only rotate to aim
    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(omega));

    boolean ready = shooter.flywheelsAtSpeed() && shooter.hoodAtTarget();

    shooter.setFeedEnabled(ready);
    indexer.setWantedState(
        ready ? IndexerSubsystem.WantedState.INDEX
              : IndexerSubsystem.WantedState.IDLE
    );
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setFeedEnabled(false);
    shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
    indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
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