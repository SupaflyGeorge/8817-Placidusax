// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.commands.autos.DepotSingleSwipe;
import frc.robot.commands.autos.DepotSingleSwipeRed;
import frc.robot.commands.autos.DriveStraight;
import frc.robot.commands.autos.Hub;
import frc.robot.commands.autos.OutpostDoubleSwipe;
import frc.robot.commands.autos.OutpostSingleSwipe;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    public final Vision vision = new Vision();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

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

    private void configureAutoChooser() {
        autoChooser.addOption(
            "Drive Straight",
            DriveStraight.build(drivetrain)
        );

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

        autoChooser.addOption(
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

        autoChooser.addOption(
            "Depot Single",
            DepotSingleSwipe.build(
                drivetrain,
                shooter,
                indexer,
                intake,
                intakePivot,
                vision,
                MaxAngularRate
            )
        );

        autoChooser.addOption(
            "Hub",
            Hub.build(
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
            "Depot Single Red",
            DepotSingleSwipeRed.build(
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

        driver.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        // Driver controls
        driver.rightBumper()
            .onTrue(new InstantCommand(() ->
                shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT)))
            .onFalse(new InstantCommand(() ->
                shooter.setWantedState(ShooterSubsystem.WantedState.IDLE)));

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

        driver.leftTrigger()
            .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE)))
            .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        driver.leftBumper()
            .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.SPIT)))
            .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        driver.a()
            .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY)))
            .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        driver.b()
            .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.STOW)))
            .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        driver.x()
            .whileTrue(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX)))
            .whileFalse(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE)));

        driver.y().onTrue(new InstantCommand(() -> shooter.increaseShooterSpeed()));
        driver.povLeft().onTrue(new InstantCommand(() -> shooter.decreaseShooterSpeed()));
        driver.povRight().onTrue(new InstantCommand(() -> shooter.resetShooterOffset()));

        // Operator controls
        operator.y()
            .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.DEPLOY)))
            .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        operator.a()
            .whileTrue(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.STOW)))
            .whileFalse(Commands.runOnce(() -> intakePivot.setWantedState(IntakePivotSubsystem.WantedState.IDLE)));

        operator.x()
            .whileTrue(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.INDEX)))
            .whileFalse(Commands.runOnce(() -> indexer.setWantedState(IndexerSubsystem.WantedState.IDLE)));

        operator.leftTrigger()
            .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.SPIT)))
            .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        operator.rightTrigger()
            .whileTrue(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.INTAKE)))
            .whileFalse(Commands.runOnce(() -> intake.setWantedState(IntakeSubsystem.WantedState.IDLE)));

        // Copy current table shot into manual fallback values
        operator.back().onTrue(
            new InstantCommand(() -> shooter.syncManualToCurrentTargets())
        );

        // Manual fallback tuning
        operator.povUp().whileTrue(
            Commands.run(() -> shooter.adjustManualHood(+0.01))
        );
        operator.povDown().whileTrue(
            Commands.run(() -> shooter.adjustManualHood(-0.01))
        );
        operator.povRight().onTrue(
            new InstantCommand(() -> shooter.adjustManualShooterRps(+1.0))
        );
        operator.povLeft().onTrue(
            new InstantCommand(() -> shooter.adjustManualShooterRps(-1.0))
        );

        // RB = normal table shot
        operator.rightBumper().onTrue(
            new InstantCommand(() -> {
                shooter.setFeedEnabled(false);
                shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT);
            })
        );

        operator.rightBumper().onFalse(
            new InstantCommand(() -> {
                if (operator.leftBumper().getAsBoolean()) {
                    shooter.setFeedEnabled(true);
                    shooter.setWantedState(ShooterSubsystem.WantedState.MANUAL_SHOT);
                } else {
                    shooter.setFeedEnabled(false);
                    shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
                }
            })
        );

        // LB = manual fallback shot
        operator.leftBumper().onTrue(
            new InstantCommand(() -> {
                shooter.setFeedEnabled(true);
                shooter.setWantedState(ShooterSubsystem.WantedState.MANUAL_SHOT);
            })
        );

        operator.leftBumper().onFalse(
            new InstantCommand(() -> {
                shooter.setFeedEnabled(false);
                if (operator.rightBumper().getAsBoolean()) {
                    shooter.setWantedState(ShooterSubsystem.WantedState.PREPARE_SHOT);
                } else {
                    shooter.setWantedState(ShooterSubsystem.WantedState.IDLE);
                }
            })
        );
    }

    public void periodic() {
        Logger.recordOutput("AutoDebug/Test", true);
        vision.updateVisionPose(drivetrain);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}