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
 * Hub auto: drive to mid-field, intake a piece near the hub, then shoot.
 * Simple one-cycle auto for when we start near the center of the field.
 */
public class Hub {
  private Hub() {}

  private static Command holdPivotDeployed(IntakePivotSubsystem intakePivot) {
    return Commands.startEnd(
        () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY),
        () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE), intakePivot);
  }

  public static Command build(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
      IndexerSubsystem indexer, IntakeSubsystem intake, IntakePivotSubsystem intakePivot,
      Vision vision, double maxAngularRateRps) {

    PathPlannerPath toZone, come;
    try {
      toZone = PathPlannerPath.fromPathFile("MID");
      come = PathPlannerPath.fromPathFile("Come");
    } catch (Exception e) { e.printStackTrace(); return Commands.none(); }

    return Commands.sequence(
        Commands.runOnce(() -> {
          Optional<Pose2d> startOpt = toZone.getStartingHolonomicPose();
          if (startOpt.isPresent()) {
            Pose2d start = startOpt.get();
            boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
            if (isRed) start = FlippingUtil.flipFieldPose(start);
            drivetrain.resetPose(start);
          }
        }),

        // Drive out, intake, come back
        Commands.deadline(
            Commands.sequence(
                Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE), intake),
                AutoBuilder.followPath(toZone),
                Commands.waitSeconds(0.5),
                AutoBuilder.followPath(come),
                Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake)),
            holdPivotDeployed(intakePivot)),

        // Shoot
        new AutoShootAlignedCommand(drivetrain, shooter, indexer, vision, intakePivot, maxAngularRateRps).withTimeout(2.5));
  }
}