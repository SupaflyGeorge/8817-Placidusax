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
    PathPlannerPath back;

    try {
      toZone = PathPlannerPath.fromPathFile("Travel 1 Dep");
      shoot = PathPlannerPath.fromPathFile("Shoot Dep");
      homeDepot = PathPlannerPath.fromPathFile("HOME DEPOT");
      back = PathPlannerPath.fromPathFile("Back");
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }

    return Commands.sequence(

        // Reset pose from the starting pose of the first path
        Commands.runOnce(() -> {
          Optional<Pose2d> startingPoseOpt = toZone.getStartingHolonomicPose();
          if (startingPoseOpt.isPresent()) {
            Pose2d startingPose = startingPoseOpt.get();

            boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

            if (isRed) {
              startingPose = FlippingUtil.flipFieldPose(startingPose);
            }

            drivetrain.resetPose(startingPose);
          }
        }),

        // PATH 1
        Commands.sequence(
            Commands.runOnce(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                intake),

            Commands.deadline(
                AutoBuilder.followPath(toZone),

                Commands.sequence(
                    Commands.waitSeconds(0.35),
                    Commands.startEnd(
                        () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY),
                        () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE),
                        intakePivot))),

            Commands.waitSeconds(1.0),

            Commands.runOnce(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE),
                intake)),

        // PATH 2
        Commands.deadline(
            AutoBuilder.followPath(shoot),
            Commands.startEnd(
                () -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE),
                () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE),
                intake)),

        // Make sure intake is off
        Commands.runOnce(
            () -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE),
            intake),

        // Shoot
        new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            intakePivot,
            maxAngularRateRps).withTimeout(3.0),

        // Re-deploy pivot after shot
        Commands.sequence(
            Commands.waitSeconds(0.35),
            Commands.startEnd(
                () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY),
                () -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE),
                intakePivot)),

        // Drive home while spinning shooter up
        Commands.deadline(
            AutoBuilder.followPath(homeDepot),
            Commands.startEnd(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT),
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE),
                shooter)),

        Commands.waitSeconds(0.35),

        Commands.deadline(
            AutoBuilder.followPath(back),
            Commands.startEnd(
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT),
                () -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE),
                shooter)),

        // Final shot
        new AutoShootAlignedCommand(
            drivetrain,
            shooter,
            indexer,
            vision,
            intakePivot,
            maxAngularRateRps).withTimeout(3.0));
  }
}