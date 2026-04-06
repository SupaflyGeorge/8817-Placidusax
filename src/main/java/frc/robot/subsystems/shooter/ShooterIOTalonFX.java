package frc.robot.subsystems.shooter;

import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/**
 * TalonFX implementation for the shooter: top flywheel, bottom flywheel, and feeder.
 * All three use VelocityVoltage (on-motor PID + kV feedforward) for precise RPM control.
 */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX top = new TalonFX(Constants.ShooterConstants.TOP_ID);
  private final TalonFX bottom = new TalonFX(Constants.ShooterConstants.BOTTOM_ID);
  private final TalonFX feeder = new TalonFX(Constants.ShooterConstants.FEEDER_ID);
  private final List<TalonFX> motors = List.of(top, bottom, feeder);

  // Velocity control requests (reused every loop to avoid allocations)
  private final VelocityVoltage topVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage botVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage feederVelCtrl = new VelocityVoltage(0).withEnableFOC(true);

  // Telemetry signals
  private final StatusSignal<AngularVelocity> topVel = top.getVelocity();
  private final StatusSignal<AngularVelocity> botVel = bottom.getVelocity();
  private final StatusSignal<AngularVelocity> feedVel = feeder.getVelocity();
  private final StatusSignal<Voltage> topVolts = top.getMotorVoltage();
  private final StatusSignal<Voltage> botVolts = bottom.getMotorVoltage();
  private final StatusSignal<Voltage> feederVolts = feeder.getMotorVoltage();
  private final StatusSignal<Current> topAmps = top.getSupplyCurrent();
  private final StatusSignal<Current> botAmps = bottom.getSupplyCurrent();
  private final StatusSignal<Current> feederAmps = feeder.getSupplyCurrent();
  private final StatusSignal<Temperature> topTemp = top.getDeviceTemp();
  private final StatusSignal<Temperature> botTemp = bottom.getDeviceTemp();
  private final StatusSignal<Temperature> feederTemp = feeder.getDeviceTemp();

  public ShooterIOTalonFX() {
    // Configure each motor with PID gains, current limits, and direction
    TalonFXConfiguration topCfg = buildFlywheelConfig(Constants.ShooterConstants.TOP_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);
    TalonFXConfiguration botCfg = buildFlywheelConfig(Constants.ShooterConstants.BOTTOM_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);
    TalonFXConfiguration feedCfg = buildFlywheelConfig(Constants.ShooterConstants.FEEDER_INVERTED,
        Constants.ShooterConstants.FEEDER_SUPPLY_LIMIT_A);

    top.getConfigurator().apply(topCfg);
    bottom.getConfigurator().apply(botCfg);
    feeder.getConfigurator().apply(feedCfg);

    BaseStatusSignal.setUpdateFrequencyForAll(50,
        topVel, botVel, topVolts, botVolts, feederVolts,
        topAmps, botAmps, feederAmps, topTemp, botTemp, feederTemp);
  }

  /** Helper to build a flywheel TalonFX config with shared PID gains. */
  private TalonFXConfiguration buildFlywheelConfig(boolean inverted, double currentLimit) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
    cfg.Slot0.kP = Constants.ShooterConstants.kP;
    cfg.Slot0.kI = Constants.ShooterConstants.kI;
    cfg.Slot0.kD = Constants.ShooterConstants.kD;
    cfg.Slot0.kV = Constants.ShooterConstants.kV;
    cfg.Slot0.kS = Constants.ShooterConstants.kS;
    cfg.Slot0.kA = Constants.ShooterConstants.kA;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = currentLimit;
    cfg.MotorOutput.Inverted = inverted
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    return cfg;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topVel, botVel, topVolts, botVolts, feederVolts,
        topAmps, botAmps, feederAmps, topTemp, botTemp, feederTemp);

    inputs.topVelocityRps = topVel.getValueAsDouble();
    inputs.bottomVelocityRps = botVel.getValueAsDouble();
    inputs.feederVelocityRps = feedVel.getValueAsDouble();
    inputs.topAppliedVolts = topVolts.getValueAsDouble();
    inputs.bottomAppliedVolts = botVolts.getValueAsDouble();
    inputs.feederAppliedVolts = feederVolts.getValueAsDouble();
    inputs.topCurrentAmps = topAmps.getValueAsDouble();
    inputs.bottomCurrentAmps = botAmps.getValueAsDouble();
    inputs.feederCurrentAmps = feederAmps.getValueAsDouble();
    inputs.topTempC = topTemp.getValueAsDouble();
    inputs.bottomTempC = botTemp.getValueAsDouble();
    inputs.feederTempC = feederTemp.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocityRps(double topRps, double bottomRps, double feedRps) {
    top.setControl(topVelCtrl.withVelocity(topRps));
    bottom.setControl(botVelCtrl.withVelocity(bottomRps));
    feeder.setControl(feederVelCtrl.withVelocity(feedRps));
  }

  @Override
  public void setFlywheelVoltage(double topVoltsOut, double bottomVoltsOut, double feedVoltsOut) {
    top.setVoltage(topVoltsOut);
    bottom.setVoltage(bottomVoltsOut);
    feeder.setVoltage(feedVoltsOut);
  }

  @Override
  public void stop() { motors.forEach(TalonFX::stopMotor); }
}
