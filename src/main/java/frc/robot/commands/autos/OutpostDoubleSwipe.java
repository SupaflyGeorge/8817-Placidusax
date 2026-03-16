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

public class OutpostDoubleSwipe {
  private OutpostDoubleSwipe() {}

  public static Command build(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      IntakePivotSubsystem intakePivot,
      Vision vision,
      double maxAngularRateRps) {

    PathPlannerPath collectSwipe1;
    PathPlannerPath travelSwipe1;
    PathPlannerPath collectSwipe2;
    PathPlannerPath swipe2COMP;

    try {
      collectSwipe1 = PathPlannerPath.fromPathFile("Collect Swipe 1");
      travelSwipe1 = PathPlannerPath.fromPathFile("Travel Swipe 1");
      collectSwipe2 = PathPlannerPath.fromPathFile("Collect Swipe 2");
      swipe2COMP = PathPlannerPath.fromPathFile("Swipe 2 COMP");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return Commands.sequence(

        // PATH 1
        Commands.deadline(
            AutoBuilder.followPath(collectSwipe1),

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

        AutoBuilder.followPath(travelSwipe1),

        new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            intakePivot,
            maxAngularRateRps
        ).withTimeout(2.5),
        

        // PATH 2
        Commands.deadline(
            AutoBuilder.followPath(collectSwipe2),

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

        AutoBuilder.followPath(swipe2COMP),

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