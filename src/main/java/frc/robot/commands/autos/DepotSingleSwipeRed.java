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

public class DepotSingleSwipeRed {
  private DepotSingleSwipeRed() {}

  public static Command build(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      IntakePivotSubsystem intakePivot,
      Vision vision,
      double maxAngularRateRps) {

    PathPlannerPath tooZone;
    PathPlannerPath shoott;
    PathPlannerPath homeDepott;
    PathPlannerPath backt;

    try {
      tooZone = PathPlannerPath.fromPathFile("Dep 1 Red");
      shoott = PathPlannerPath.fromPathFile("Dep 2 Red");
      homeDepott = PathPlannerPath.fromPathFile("Dep 3 Red");
      backt = PathPlannerPath.fromPathFile("Dep 4 Red");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return Commands.sequence(

        /*Commands.runOnce(
            () -> tooZone.getStartingHolonomicPose().ifPresent(drivetrain::resetPose)
        ),*/

        // PATH 1
        Commands.deadline(
            AutoBuilder.followPath(tooZone),

            Commands.startEnd(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE),
                intake
            ),

            Commands.sequence(
                Commands.waitSeconds(0.35),
                Commands.startEnd(
                    () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY),
                    () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE),
                    intakePivot
                )
            )
        ),

        // PATH 2
        Commands.deadline(
            AutoBuilder.followPath(shoott),

            Commands.startEnd(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE),
                intake
            )
        ),

        // stop intake
        Commands.runOnce(
            () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE),
            intake
        ),

        // shoot
        new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            intakePivot,
            maxAngularRateRps
        ).withTimeout(3.0),

        // drive home while only prepping shooter, NOT running another drivetrain command
        Commands.deadline(
            AutoBuilder.followPath(homeDepott),
            Commands.startEnd(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT),
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE),
                shooter
            )
        ),

        Commands.sequence(
                Commands.waitSeconds(0.35)
        ),

        Commands.deadline(
            AutoBuilder.followPath(backt),
            Commands.startEnd(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT),
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE),
                shooter
            )
        ),



        // then do the aligned shot after the path is complete
        new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            intakePivot,
            maxAngularRateRps
        ).withTimeout(3.0)
    );
  }
}