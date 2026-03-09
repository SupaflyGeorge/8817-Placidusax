package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;

public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX belt = new TalonFX(Constants.IndexerConstants.BELTER_ID);

  private final DutyCycleOut duty = new DutyCycleOut(0).withEnableFOC(true);

  private final StatusSignal<Voltage> volts = belt.getMotorVoltage();
  private final StatusSignal<Current> amps = belt.getSupplyCurrent();
  private final StatusSignal<Temperature> temp = belt.getDeviceTemp();

  private double lastPercent = 0.0;

  public IndexerIOTalonFX() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = Constants.IndexerConstants.BELT_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IndexerConstants.BELT_SUPPLY_LIMIT_A;

    belt.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(50, volts, amps, temp);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(volts, amps, temp);

    inputs.beltPercent = lastPercent;
    inputs.beltVolts = volts.getValueAsDouble();
    inputs.beltAmps = amps.getValueAsDouble();
    inputs.beltTempC = temp.getValueAsDouble();
  }

  @Override
  public void setBeltPercent(double percent) {
    lastPercent = percent;
    belt.setControl(duty.withOutput(percent));
  }

  @Override
  public void stop() {
    lastPercent = 0.0;
    belt.stopMotor();
  }
}