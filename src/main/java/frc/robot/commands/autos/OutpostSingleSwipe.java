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

public class OutpostSingleSwipe {
  private OutpostSingleSwipe() {}

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
      toZone = PathPlannerPath.fromPathFile("To Zone");
      collect = PathPlannerPath.fromPathFile("Collect");
      travel = PathPlannerPath.fromPathFile("Travel");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return Commands.sequence(

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

        // PATH 2
        Commands.deadline(
            AutoBuilder.followPath(collect),

            Commands.startEnd(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE),
                intake
            )
        ),

        // PATH 3
        Commands.deadline(
            AutoBuilder.followPath(travel),

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

        // stow intake
        Commands.startEnd(
            () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.STOW),
            () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE),
            intakePivot
        ).withTimeout(1.0),

        // shoot
        new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            maxAngularRateRps
        ).withTimeout(2.5)
    );
  }
}