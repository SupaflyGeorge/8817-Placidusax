package frc.robot.subsystems.intake;

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

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX roller = new TalonFX(Constants.IntakeConstants.ROLLER_ID);

  private final DutyCycleOut duty = new DutyCycleOut(0).withEnableFOC(true);

  private final StatusSignal<Voltage> volts = roller.getMotorVoltage();
  private final StatusSignal<Current> amps = roller.getSupplyCurrent();
  private final StatusSignal<Temperature> temp = roller.getDeviceTemp();

  private double lastPercent = 0.0;

  public IntakeIOTalonFX() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = Constants.IntakeConstants.ROLLER_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.ROLLER_SUPPLY_LIMIT_A;

    roller.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(50, volts, amps, temp);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(volts, amps, temp);

    inputs.rollerPercent = lastPercent;
    inputs.rollerVolts = volts.getValueAsDouble();
    inputs.rollerAmps = amps.getValueAsDouble();
    inputs.rollerTempC = temp.getValueAsDouble();
  }

  @Override
  public void setRollerPercent(double percent) {
    lastPercent = percent;
    roller.setControl(duty.withOutput(percent));
  }

  @Override
  public void stop() {
    lastPercent = 0.0;
    roller.stopMotor();
  }
}