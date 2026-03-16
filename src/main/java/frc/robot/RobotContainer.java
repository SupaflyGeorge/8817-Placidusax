// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakepivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.ShootOnMoveCommand;
import frc.robot.commands.autos.OutpostDoubleSwipe;
import frc.robot.commands.autos.OutpostSingleSwipe;
import frc.robot.commands.AutoStateCommand;
import frc.robot.commands.AutoShootAlignedCommand;
import frc.robot.generated.TunerConstants;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    public final Vision vision = new Vision();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    

    private void configureAutoChooser() {

    autoChooser.addOption(
        "Outpost Single Swipe",
        OutpostSingleSwipe.build(
            drivetrain,
            shooter,
            indexer,
            intake,
            intakePivot,
            vision,
            MaxAngularRate
        )
    );

    autoChooser.setDefaultOption(
        "Outpost Double Swipe",
        OutpostDoubleSwipe.build(
            drivetrain,
            shooter,
            indexer,
            intake,
            intakePivot,
            vision,
            MaxAngularRate
        )
    );

      ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
      autoTab.add(autoChooser);
}
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        drivetrain.configurePathPlanner();
        configureAutoChooser();
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * MaxSpeed)
                    .withVelocityY(driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on right stick press.
        driver.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Driver Controls
        /*driver.y().whileTrue(
            drivetrain.applyRequest(() -> {
              double vx = -driver.getLeftY() * MaxSpeed;
              double vy = -driver.getLeftX() * MaxSpeed;

              double omegaCmd = vision.calcAimOmegaRadPerSec();
              omegaCmd = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omegaCmd));

              return drive
                  .withVelocityX(vx)
                  .withVelocityY(vy)
                  .withRotationalRate(omegaCmd);
            })
        );*/

        // Hold = prepare shot (spin + aim hood), release = idle (hood returns to 0)
        driver.rightBumper()
          .whileTrue(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT)))
          .whileFalse(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE)));

        // Shoot On The Move
        driver.rightTrigger().whileTrue(
          new ShootOnMoveCommand(
            drivetrain,
            shooter,
            vision,
            indexer,
            intakePivot,
            () -> driver.getLeftY(),
            () -> driver.getLeftX(),
            () -> -driver.getRightX(),
            MaxSpeed,
            MaxAngularRate
          )
        );

        // Nudge hood UP
        driver.povUp()
         .whileTrue(Commands.run(() -> shooter.adjustHoodManual(+0.01)));

        // Nudge hood DOWN
        driver.povDown()
          .whileTrue(Commands.run(() -> shooter.adjustHoodManual(-0.01)));

        // Intake roller: hold left trigger to intake, release = idle
        driver.leftTrigger()
          .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE)))
          .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        driver.leftBumper()
          .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.SPIT)))
          .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        // Pivot: A = deploy, B = stow
        driver.a()
        .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY)))
        .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        driver.b()
        .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.STOW)))
        .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        driver.x()
          .whileTrue(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX)))
          .whileFalse(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE)));

        driver.y().onTrue(
          new InstantCommand(() -> shooter.increaseShooterSpeed()));

        driver.povLeft().onTrue(
          new InstantCommand(() -> shooter.decreaseShooterSpeed()));

        driver.povRight().onTrue(
          new InstantCommand(() -> shooter.resetShooterOffset()));

        //  Chris
        operator.y()
        .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY)))
        .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        operator.a()
        .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.STOW)))
        .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        operator.x()
        .whileTrue(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX)))
        .whileFalse(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE)));

        operator.y()
        .whileTrue(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT)))
        .whileFalse(Commands.runOnce(() -> shooter.setWantedState(ShooterSubsystem.WantedState.IDLE)));

        operator.leftTrigger()
        .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.SPIT)))
        .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        operator.rightTrigger()
          .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE)))
          .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        operator.povUp()
        .whileTrue(Commands.run(() -> shooter.adjustHoodManual(+0.01)));

        operator.povDown()
        .whileTrue(Commands.run(() -> shooter.adjustHoodManual(-0.01)));

  
          }

    public void periodic() {
      Logger.recordOutput("AutoDebug/Test", true);
      vision.updateVisionPose(drivetrain);
    }

    public Command getAutonomousCommand() {
  return autoChooser.getSelected();
}
}