package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Our swerve drivetrain subsystem — wraps CTRE's TunerSwerveDrivetrain
 * and implements WPILib's Subsystem interface so it works with the CommandScheduler.
 *
 * Key responsibilities:
 *   - Configure PathPlanner for autonomous path following
 *   - Apply operator perspective (field-centric "forward" flips for red alliance)
 *   - Provide helper methods for SysId characterization
 *   - Override vision measurement timestamps for CTRE compatibility
 *   - Run a simulation thread when not on real hardware
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 250Hz sim update
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Field-centric "forward" direction per alliance
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    // PathPlanner setup state
    private boolean m_pathPlannerConfigured = false;
    private boolean m_attemptedPathPlannerConfig = false;

    // Orchestra state
    private final Orchestra m_orchestra = new Orchestra();
    private boolean m_musicLoaded = false;

    // The swerve request used by PathPlanner to command robot-relative speeds
    private final SwerveRequest.ApplyRobotSpeeds m_ppRobotSpeedsRequest =
        new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    /**
     * Configure PathPlanner's AutoBuilder. This reads the robot config from
     * deploy/pathplanner/settings.json and sets up the path-following controller.
     * Call this once in RobotContainer's constructor.
     */
    public void configurePathPlanner() {
        if (m_pathPlannerConfigured) return;
        if (m_attemptedPathPlannerConfig && !m_pathPlannerConfigured) return;
        m_attemptedPathPlannerConfig = true;

        try {
            final RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.getState().Pose,       // pose supplier
                this::resetPose,                   // pose resetter
                () -> this.getState().Speeds,      // speeds supplier
                speeds -> this.setControl(m_ppRobotSpeedsRequest.withSpeeds(speeds)), // speed consumer
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),  // translation PID
                    new PIDConstants(5.0, 0.0, 0.0)   // rotation PID
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // flip for red?
                this
            );
            m_pathPlannerConfigured = true;
            DriverStation.reportWarning("PathPlanner configured OK.", false);
        } catch (Exception e) {
            m_pathPlannerConfigured = false;
            DriverStation.reportError(
                "PathPlanner configure FAILED. Export GUI settings to src/main/deploy/pathplanner/. "
                + "Error: " + e.getMessage(), e.getStackTrace());
        }
    }

    public boolean isPathPlannerConfigured() { return m_pathPlannerConfigured; }

    /** Build a named auto from PathPlanner, with error handling. */
    public Command getAutoCommand(String autoName) {
        configurePathPlanner();
        if (!m_pathPlannerConfigured) {
            DriverStation.reportError("Auto '" + autoName + "' requested but PathPlanner is NOT configured.", false);
            return Commands.none();
        }
        try { return new PathPlannerAuto(autoName); }
        catch (Exception e) {
            DriverStation.reportError("Failed to create PathPlannerAuto '" + autoName + "': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public void configureOrchestra() {
        if (m_musicLoaded) return;

        for (var module : getModules()) {
            m_orchestra.addInstrument(module.getDriveMotor());
            m_orchestra.addInstrument(module.getSteerMotor());
        }

        String musicPath = new File(Filesystem.getDeployDirectory(), "input.chrp").getAbsolutePath();
        var status = m_orchestra.loadMusic(musicPath);

        if (status.isOK()) {
            m_musicLoaded = true;
            DriverStation.reportWarning("Loaded CHRP: " + musicPath, false);
        } else {
            DriverStation.reportError("Failed to load CHRP: " + status.toString(), false);
        }
    }

    public void playMusic() {
        configureOrchestra();
        if (m_musicLoaded) {
            m_orchestra.play();
        }
    }

    public void stopMusic() {
        m_orchestra.stop();
    }

    public void pauseMusic() {
        m_orchestra.pause();
    }

    public boolean isMusicPlaying() {
        return m_orchestra.isPlaying();
    }

    // --- SysId characterization ---
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this));
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // --- Constructors ---
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimThread();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    /** Wrap a SwerveRequest supplier into a Command (the default drive command pattern). */
    public Command applyRequest(Supplier<SwerveRequest> request) { return run(() -> this.setControl(request.get())); }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { return m_sysIdRoutineToApply.quasistatic(direction); }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) { return m_sysIdRoutineToApply.dynamic(direction); }

    // --- Speed helpers ---
    public ChassisSpeeds getRobotRelativeSpeeds() { return getState().Speeds; }
    public ChassisSpeeds getFieldRelativeSpeeds() { return ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getState().Pose.getRotation()); }
    public double getFieldSpeedMagnitudeMps() { ChassisSpeeds s = getFieldRelativeSpeeds(); return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond); }

    /**
     * Apply operator perspective every loop while disabled (alliance may change).
     * This sets which direction is "forward" for field-centric driving.
     */
    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /** Simulation thread: ticks physics at 250Hz for accurate sim behavior. */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // --- Vision measurement overrides ---
    // CTRE uses a different time base than WPILib, so we convert FPGA timestamps
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}