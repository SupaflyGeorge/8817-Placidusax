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

public class Hub {
  private Hub() {}

  public static Command build(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      IntakePivotSubsystem intakePivot,
      Vision vision,
      double maxAngularRateRps) {

    PathPlannerPath toZone;
    PathPlannerPath collect;
    PathPlannerPath travel;

    try {
      toZone = PathPlannerPath.fromPathFile("MID");
      
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return Commands.sequence(

    Commands.runOnce(() ->
            toZone.getStartingHolonomicPose().ifPresent(drivetrain::resetPose)
        ),

        // PATH 1
        Commands.deadline(
            AutoBuilder.followPath(toZone),

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

        // shoot
        new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            intakePivot,
            maxAngularRateRps
        ).withTimeout(2.5)
    );
  }
}