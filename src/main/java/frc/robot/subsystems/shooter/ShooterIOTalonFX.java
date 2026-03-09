package frc.robot.subsystems.shooter;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX top = new TalonFX(Constants.ShooterConstants.TOP_ID);
  private final TalonFX bottom = new TalonFX(Constants.ShooterConstants.BOTTOM_ID);
  private final TalonFX feeder = new TalonFX(Constants.ShooterConstants.FEEDER_ID);

  private final List<TalonFX> motors = List.of(top, bottom, feeder);

  private final VelocityVoltage topVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage botVelCtrl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage feederVelCtrl = new VelocityVoltage(0).withEnableFOC(true);

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
    // Base flywheel config
    TalonFXConfiguration topCfg = new TalonFXConfiguration();
    topCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topCfg.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;

    topCfg.Slot0.kP = Constants.ShooterConstants.kP;
    topCfg.Slot0.kI = Constants.ShooterConstants.kI;
    topCfg.Slot0.kD = Constants.ShooterConstants.kD;
    topCfg.Slot0.kV = Constants.ShooterConstants.kV;
    topCfg.Slot0.kS = Constants.ShooterConstants.kS;
    topCfg.Slot0.kA = Constants.ShooterConstants.kA;

    topCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    topCfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.SUPPLY_LIMIT_A;

    topCfg.MotorOutput.Inverted = Constants.ShooterConstants.TOP_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration botCfg = new TalonFXConfiguration();
    botCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    botCfg.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;

    botCfg.Slot0.kP = Constants.ShooterConstants.kP;
    botCfg.Slot0.kI = Constants.ShooterConstants.kI;
    botCfg.Slot0.kD = Constants.ShooterConstants.kD;
    botCfg.Slot0.kV = Constants.ShooterConstants.kV;
    botCfg.Slot0.kS = Constants.ShooterConstants.kS;
    botCfg.Slot0.kA = Constants.ShooterConstants.kA;

    botCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    botCfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.SUPPLY_LIMIT_A;

    botCfg.MotorOutput.Inverted = Constants.ShooterConstants.BOTTOM_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration feedCfg = new TalonFXConfiguration();
    feedCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feedCfg.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;

    feedCfg.Slot0.kP = Constants.ShooterConstants.kP;
    feedCfg.Slot0.kI = Constants.ShooterConstants.kI;
    feedCfg.Slot0.kD = Constants.ShooterConstants.kD;
    feedCfg.Slot0.kV = Constants.ShooterConstants.kV;
    feedCfg.Slot0.kS = Constants.ShooterConstants.kS;
    feedCfg.Slot0.kA = Constants.ShooterConstants.kA;

    feedCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    feedCfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.FEEDER_SUPPLY_LIMIT_A;

    feedCfg.MotorOutput.Inverted = Constants.ShooterConstants.FEEDER_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    top.getConfigurator().apply(topCfg);
    bottom.getConfigurator().apply(botCfg);
    feeder.getConfigurator().apply(feedCfg);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        topVel, botVel, topVolts, botVolts, feederVolts,
        topAmps, botAmps, feederAmps,
        topTemp, botTemp, feederTemp
    );
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topVel, botVel, topVolts, botVolts, feederVolts,
        topAmps, botAmps, feederAmps,
        topTemp, botTemp, feederTemp
    );

    inputs.topVelocityRps = topVel.getValueAsDouble();
    inputs.bottomVelocityRps = botVel.getValueAsDouble();
    inputs.feederVelocityRps =feedVel.getValueAsDouble();

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
  public void stop() {
    motors.forEach(TalonFX::stopMotor);
  }

}