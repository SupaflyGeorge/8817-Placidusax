package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Generic auto command that sets one or more subsystem states.
 * Used in auto routines when you need to start/stop mechanisms at a specific point.
 *
 * Example: AutoStateCommand(INTAKE, ...) starts the intake rollers.
 * The SHOOT action is special: it waits for flywheels to reach speed before feeding.
 *
 * Publishes debug info to SmartDashboard so you can see which auto step is active.
 */
public class AutoStateCommand extends Command {

  public enum AutoAction {
    PREPARE_SHOT, SHOOT, INTAKE, SPIT,
    DEPLOY_INTAKE, STOW_INTAKE, INDEX, STOP_ALL
  }

  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final IntakeSubsystem intake;
  private final IntakePivotSubsystem pivot;
  private final AutoAction action;
  private final Set<Subsystem> requirements;
  private final Timer timer = new Timer();

  public AutoStateCommand(AutoAction action, ShooterSubsystem shooter,
      IndexerSubsystem indexer, IntakeSubsystem intake, IntakePivotSubsystem pivot) {
    this.action = action;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.pivot = pivot;
    this.requirements = Set.of(shooter, indexer, intake, pivot);
  }

  @Override
  public void initialize() {
    timer.restart();
    SmartDashboard.putString("AutoMarker/ActiveCommand", action.name());
    SmartDashboard.putBoolean("AutoMarker/CommandRunning", true);
    SmartDashboard.putNumber("AutoMarker/CommandStartTime", Timer.getFPGATimestamp());

    switch (action) {
      case PREPARE_SHOT  -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT);
      case SHOOT         -> { shooter.setWantedState(ShooterSubsystem.WantedState.SHOOTING);
                              shooter.setFeedEnabled(false);
                              indexer.setWantedState(IndexerSubsystem.WantedState.IDLE); }
      case INTAKE        -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE);
      case SPIT          -> intake.setWantedState(IntakeSubsystem.WantedState.SPIT);
      case DEPLOY_INTAKE -> pivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY);
      case STOW_INTAKE   -> pivot.setWantedState(IntakePivotSubsystem.WantedState.STOW);
      case INDEX         -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX);
      case STOP_ALL      -> { shooter.setFeedEnabled(false);
                              shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
                              indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
                              intake.setWantedState(IntakeSubsystem.WantedState.IDLE);
                              pivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE); }
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putString("AutoMarker/ActiveCommand", action.name());
    SmartDashboard.putNumber("AutoMarker/CommandElapsed", timer.get());

    // SHOOT is special: only feed when flywheels are at speed
    if (action == AutoAction.SHOOT) {
      boolean ready = shooter.flywheelsAtSpeed();
      shooter.setFeedEnabled(ready);
      indexer.setWantedState(ready ? IndexerSubsystem.WantedState.INDEX : IndexerSubsystem.WantedState.IDLE);
      SmartDashboard.putBoolean("AutoMarker/ShootReady", ready);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (action == AutoAction.SHOOT) {
      shooter.setFeedEnabled(false);
      indexer.setWantedState(IndexerSubsystem.WantedState.IDLE);
    }
    SmartDashboard.putBoolean("AutoMarker/CommandRunning", false);
    SmartDashboard.putBoolean("AutoMarker/CommandInterrupted", interrupted);
  }

  @Override public boolean isFinished() { return false; }
  @Override public Set<Subsystem> getRequirements() { return requirements; }
}
