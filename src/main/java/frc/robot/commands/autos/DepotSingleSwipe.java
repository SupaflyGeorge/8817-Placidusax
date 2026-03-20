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

public class DepotSingleSwipe {
  private DepotSingleSwipe() {}

  public static Command build(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      IntakePivotSubsystem intakePivot,
      Vision vision,
      double maxAngularRateRps) {

    PathPlannerPath toZone;
    PathPlannerPath shoot;
    PathPlannerPath homeDepot;
    

    try {
      toZone = PathPlannerPath.fromPathFile("Travel 1 Dep");
      shoot = PathPlannerPath.fromPathFile("Shoot Dep");
      homeDepot = PathPlannerPath.fromPathFile("HOME DEPOT");
    
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

        // PATH 2
        Commands.deadline(
            AutoBuilder.followPath(shoot),

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

        Commands.deadline(
            AutoBuilder.followPath(homeDepot),

            new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            intakePivot,
            maxAngularRateRps
        ).withTimeout(10.0)
        )
    );
  }
}