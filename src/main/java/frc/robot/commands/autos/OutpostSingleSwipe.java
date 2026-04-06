package frc.robot.commands.autos;

import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoShootAlignedCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

/**
 * Outpost Single Swipe auto:
 *   1. Drive to the scoring zone while intaking with pivot deployed
 *   2. Collect at the zone, then travel back
 *   3. Auto-aim and shoot
 *   4. Drive for a second intake cycle (half path)
 *
 * Uses alliance-aware pose reset so paths flip correctly for red side.
 */
public class OutpostSingleSwipe {
  private OutpostSingleSwipe() {}

  private static Command holdPivotDeployed(IntakePivotSubsystem intakePivot) {
    return Commands.startEnd(
        () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY),
        () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE),
        intakePivot);
  }

  public static Command build(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
      IndexerSubsystem indexer, IntakeSubsystem intake, IntakePivotSubsystem intakePivot,
      Vision vision, double maxAngularRateRps) {

    PathPlannerPath toZone, shoot, homeDepot, half;
    try {
      toZone = PathPlannerPath.fromPathFile("To Zone");
      shoot = PathPlannerPath.fromPathFile("Collect");
      homeDepot = PathPlannerPath.fromPathFile("Travel");
      half = PathPlannerPath.fromPathFile("Half");
    } catch (Exception e) { e.printStackTrace(); return Commands.none(); }

    return Commands.sequence(
        // Reset pose (flip for red alliance)
        Commands.runOnce(() -> {
          Optional<Pose2d> startOpt = toZone.getStartingHolonomicPose();
          if (startOpt.isPresent()) {
            Pose2d start = startOpt.get();
            boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
            if (isRed) start = FlippingUtil.flipFieldPose(start);
            drivetrain.resetPose(start);
          }
        }),

        // Phase 1: drive to zone + collect (intake running, pivot deployed)
        Commands.deadline(
            Commands.sequence(
                Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE), intake),
                Commands.deadline(AutoBuilder.followPath(toZone), Commands.waitSeconds(0.35)),
                Commands.waitSeconds(0.35),
                Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake),
                Commands.deadline(AutoBuilder.followPath(shoot),
                    Commands.startEnd(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                        () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake)),
                Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake)),
            holdPivotDeployed(intakePivot)),

        // Phase 2: travel back while intaking
        Commands.deadline(
            Commands.sequence(
                Commands.deadline(AutoBuilder.followPath(homeDepot),
                    Commands.startEnd(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                        () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake)),
                Commands.waitSeconds(0.35)),
            holdPivotDeployed(intakePivot)),

        // Phase 3: auto-aim and shoot
        new AutoShootAlignedCommand(drivetrain, shooter, indexer, vision, intakePivot, maxAngularRateRps).withTimeout(5.0),

        // Phase 4: second intake run
        Commands.deadline(
            Commands.deadline(AutoBuilder.followPath(half),
                Commands.startEnd(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                    () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake)),
            holdPivotDeployed(intakePivot)));
  }
}