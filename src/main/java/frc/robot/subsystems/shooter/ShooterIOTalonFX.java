package frc.robot.subsystems.shooter;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX top = new TalonFX(Constants.ShooterConstants.TOP_ID);
  private final TalonFX mid = new TalonFX(Constants.ShooterConstants.MID_ID);
  private final TalonFX bottom = new TalonFX(Constants.ShooterConstants.BOTTOM_ID);
  private final TalonFX feeder = new TalonFX(Constants.ShooterConstants.FEEDER_ID);

  private final List<TalonFX> motors = List.of(top, mid, bottom, feeder);

  private final VelocityVoltage topVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage botVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage feederVelCtrl = new VelocityVoltage(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> topVel = top.getVelocity();
  private final StatusSignal<AngularVelocity> midVel = mid.getVelocity();
  private final StatusSignal<AngularVelocity> botVel = bottom.getVelocity();
  private final StatusSignal<AngularVelocity> feedVel = feeder.getVelocity();

  private final StatusSignal<Voltage> topVolts = top.getMotorVoltage();
  private final StatusSignal<Voltage> midVolts = mid.getMotorVoltage();
  private final StatusSignal<Voltage> botVolts = bottom.getMotorVoltage();
  private final StatusSignal<Voltage> feederVolts = feeder.getMotorVoltage();

  private final StatusSignal<Current> topAmps = top.getSupplyCurrent();
  private final StatusSignal<Current> midAmps = mid.getSupplyCurrent();
  private final StatusSignal<Current> botAmps = bottom.getSupplyCurrent();
  private final StatusSignal<Current> feederAmps = feeder.getSupplyCurrent();

  private final StatusSignal<Temperature> topTemp = top.getDeviceTemp();
  private final StatusSignal<Temperature> midTemp = mid.getDeviceTemp();
  private final StatusSignal<Temperature> botTemp = bottom.getDeviceTemp();
  private final StatusSignal<Temperature> feederTemp = feeder.getDeviceTemp();

  public ShooterIOTalonFX() {
    TalonFXConfiguration topCfg = buildFlywheelConfig(
        Constants.ShooterConstants.TOP_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);

    TalonFXConfiguration midCfg = buildFlywheelConfig(
        Constants.ShooterConstants.MID_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);

    TalonFXConfiguration botCfg = buildFlywheelConfig(
        Constants.ShooterConstants.BOTTOM_INVERTED,
        Constants.ShooterConstants.SUPPLY_LIMIT_A);

    TalonFXConfiguration feedCfg = buildFlywheelConfig(
        Constants.ShooterConstants.FEEDER_INVERTED,
        Constants.ShooterConstants.FEEDER_SUPPLY_LIMIT_A);

    top.getConfigurator().apply(topCfg);
    mid.getConfigurator().apply(midCfg);
    bottom.getConfigurator().apply(botCfg);
    feeder.getConfigurator().apply(feedCfg);

    mid.setControl(new Follower(top.getDeviceID(), MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        topVel, midVel, botVel, feedVel,
        topVolts, midVolts, botVolts, feederVolts,
        topAmps, midAmps, botAmps, feederAmps,
        topTemp, midTemp, botTemp, feederTemp);
  }

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

    // Helps stop huge current spikes when balls load the drum.
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;

    // Smooths shooter voltage changes so the RIO/battery does not get slammed.
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.10;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.10;

    cfg.MotorOutput.Inverted = inverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    return cfg;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topVel, midVel, botVel, feedVel,
        topVolts, midVolts, botVolts, feederVolts,
        topAmps, midAmps, botAmps, feederAmps,
        topTemp, midTemp, botTemp, feederTemp);

    inputs.topVelocityRps = topVel.getValueAsDouble();
    inputs.midVelocityRps = midVel.getValueAsDouble();
    inputs.bottomVelocityRps = botVel.getValueAsDouble();
    inputs.feederVelocityRps = feedVel.getValueAsDouble();

    inputs.topAppliedVolts = topVolts.getValueAsDouble();
    inputs.midAppliedVolts = midVolts.getValueAsDouble();
    inputs.bottomAppliedVolts = botVolts.getValueAsDouble();
    inputs.feederAppliedVolts = feederVolts.getValueAsDouble();

    inputs.topCurrentAmps = topAmps.getValueAsDouble();
    inputs.midCurrentAmps = midAmps.getValueAsDouble();
    inputs.bottomCurrentAmps = botAmps.getValueAsDouble();
    inputs.feederCurrentAmps = feederAmps.getValueAsDouble();

    inputs.topTempC = topTemp.getValueAsDouble();
    inputs.midTempC = midTemp.getValueAsDouble();
    inputs.bottomTempC = botTemp.getValueAsDouble();
    inputs.feederTempC = feederTemp.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocityRps(double topRps, double bottomRps, double feedRps) {
    top.setControl(topVelCtrl.withVelocity(topRps));
    bottom.setControl(botVelCtrl.withVelocity(bottomRps));
    feeder.setControl(feederVelCtrl.withVelocity(feedRps));

    mid.setControl(new Follower(top.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void setFlywheelVoltage(double topVoltsOut, double bottomVoltsOut, double feedVoltsOut) {
    top.setVoltage(topVoltsOut);
    bottom.setVoltage(bottomVoltsOut);
    feeder.setVoltage(feedVoltsOut);

    mid.setControl(new Follower(top.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void stop() {
    motors.forEach(TalonFX::stopMotor);
  }
}