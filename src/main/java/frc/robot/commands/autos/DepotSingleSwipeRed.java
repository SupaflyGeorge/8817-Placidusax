package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
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
 * Red-alliance-specific depot auto using hardcoded red-side paths.
 * Same strategy as DepotSingleSwipe but with paths designed for the
 * red starting position (Dep 1-4 Red).
 *
 * NOTE: Not currently registered in the auto chooser.
 */
public class DepotSingleSwipeRed {
  private DepotSingleSwipeRed() {}

  public static Command build(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
      IndexerSubsystem indexer, IntakeSubsystem intake, IntakePivotSubsystem intakePivot,
      Vision vision, double maxAngularRateRps) {

    PathPlannerPath tooZone, shoott, homeDepott, backt;
    try {
      tooZone = PathPlannerPath.fromPathFile("Dep 1 Red");
      shoott = PathPlannerPath.fromPathFile("Dep 2 Red");
      homeDepott = PathPlannerPath.fromPathFile("Dep 3 Red");
      backt = PathPlannerPath.fromPathFile("Dep 4 Red");
    } catch (Exception e) { e.printStackTrace(); return Commands.none(); }

    return Commands.sequence(
        // Path 1: drive + intake + deploy pivot
        Commands.deadline(AutoBuilder.followPath(tooZone),
            Commands.startEnd(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake),
            Commands.sequence(Commands.waitSeconds(0.35),
                Commands.startEnd(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY),
                    () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE), intakePivot))),
        // Path 2: continue collecting
        Commands.deadline(AutoBuilder.followPath(shoott),
            Commands.startEnd(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake)),
        Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE), intake),
        // First shot
        new AutoShootAlignedCommand(drivetrain, shooter, indexer, vision, intakePivot, maxAngularRateRps).withTimeout(3.0),
        // Drive home prepping shooter
        Commands.deadline(AutoBuilder.followPath(homeDepott),
            Commands.startEnd(() -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT),
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE), shooter)),
        Commands.waitSeconds(0.35),
        Commands.deadline(AutoBuilder.followPath(backt),
            Commands.startEnd(() -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT),
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE), shooter)),
        // Second shot
        new AutoShootAlignedCommand(drivetrain, shooter, indexer, vision, intakePivot, maxAngularRateRps).withTimeout(3.0));
  }
}
