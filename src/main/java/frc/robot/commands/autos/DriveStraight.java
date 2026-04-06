package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Simplest auto: resets pose to the path's start, then drives straight.
 * Good for testing that PathPlanner is working and the robot drives correctly.
 */
public class DriveStraight {
    private DriveStraight() {}

    public static Command build(CommandSwerveDrivetrain drivetrain) {
        PathPlannerPath driveStraight;
        try {
            driveStraight = PathPlannerPath.fromPathFile("New Path");
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
        return Commands.sequence(
            Commands.runOnce(() -> driveStraight.getStartingHolonomicPose().ifPresent(drivetrain::resetPose)),
            AutoBuilder.followPath(driveStraight)
        );
    }
}