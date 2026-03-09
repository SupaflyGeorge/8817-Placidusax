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

  // USE MOTOR POSITION LIKE HOOD
  private final StatusSignal<Angle> motorPos = pivotMotor.getPosition();
  private final StatusSignal<Voltage> motorVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> motorAmps = pivotMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> motorTemp = pivotMotor.getDeviceTemp();

  // only for debug
  private final StatusSignal<Angle> cancoderAbs = pivotEncoder.getAbsolutePosition();
  private final StatusSignal<AngularVelocity> cancoderVel = pivotEncoder.getVelocity();

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

    // This makes motor position report mechanism rotations instead of raw motor rotations
    cfg.Feedback.SensorToMechanismRatio = Constants.IntakePivotConstants.PIVOT_GEAR_RATIO;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivotConstants.PIVOT_SUPPLY_LIMIT_A;

    pivotMotor.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorPos,
        motorVolts,
        motorAmps,
        motorTemp,
        cancoderAbs,
        cancoderVel
    );
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorPos,
        motorVolts,
        motorAmps,
        motorTemp,
        cancoderAbs,
        cancoderVel
    );

    if (!seeded) {
      pivotEncoder.setPosition(0);
      pivotMotor.setPosition(0);
      seeded = true;
    }

    inputs.motorPercent = lastPercent;

    inputs.motorVolts = motorVolts.getValueAsDouble();
    inputs.motorAmps = motorAmps.getValueAsDouble();
    inputs.motorTempC = motorTemp.getValueAsDouble();

    // USE MOTOR POSITION AS THE REAL POSITION
    inputs.pivotPositionRot = motorPos.getValueAsDouble();

    // debug only
    inputs.pivotVelocityRps = cancoderVel.getValueAsDouble();
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