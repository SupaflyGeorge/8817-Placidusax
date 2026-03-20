package frc.robot.subsystems.intakepivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX pivotMotor = new TalonFX(Constants.IntakePivotConstants.PIVOT_ID);
  private final CANcoder pivotEncoder = new CANcoder(Constants.IntakePivotConstants.PIVOT_CANCODER_ID);

  private final DutyCycleOut duty = new DutyCycleOut(0).withEnableFOC(true);

  // Continuous TalonFX signals used for real control
  private final StatusSignal<Angle> motorPos = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> motorVel = pivotMotor.getVelocity();
  private final StatusSignal<Voltage> motorVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> motorAmps = pivotMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> motorTemp = pivotMotor.getDeviceTemp();

  // Absolute CANcoder signals for debug only
  private final StatusSignal<Angle> cancoderAbs = pivotEncoder.getAbsolutePosition();

  private double lastPercent = 0.0;
  private boolean seeded = false;

  public IntakePivotIOTalonFX() {
    pivotEncoder.getConfigurator().apply(new CANcoderConfiguration());

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.MotorOutput.Inverted =
        Constants.IntakePivotConstants.PIVOT_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Makes Talon position report mechanism rotations instead of raw motor rotations
    cfg.Feedback.SensorToMechanismRatio = Constants.IntakePivotConstants.PIVOT_GEAR_RATIO;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivotConstants.PIVOT_SUPPLY_LIMIT_A;

    pivotMotor.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorPos,
        motorVel,
        motorVolts,
        motorAmps,
        motorTemp,
        cancoderAbs);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorPos,
        motorVel,
        motorVolts,
        motorAmps,
        motorTemp,
        cancoderAbs);

    // Zero the Talon's continuous position once on boot
    if (!seeded) {
      pivotMotor.setPosition(0);
      seeded = true;
    }

    inputs.motorPercent = lastPercent;

    inputs.motorVolts = motorVolts.getValueAsDouble();
    inputs.motorAmps = motorAmps.getValueAsDouble();
    inputs.motorTempC = motorTemp.getValueAsDouble();

    // Real continuous position for control
    inputs.pivotPositionRot = motorPos.getValueAsDouble();

    // Real continuous velocity for control/debug
    inputs.pivotVelocityRps = motorVel.getValueAsDouble();

    // Absolute encoder for debug only - this will wrap
    inputs.pivotAbsPositionRot = cancoderAbs.getValueAsDouble();

    inputs.homed = seeded;
  }

  @Override
  public void setPercent(double percent) {
    lastPercent = percent;
    pivotMotor.setControl(duty.withOutput(percent));
  }

  @Override
  public void stop() {
    lastPercent = 0.0;
    pivotMotor.stopMotor();
  }
}