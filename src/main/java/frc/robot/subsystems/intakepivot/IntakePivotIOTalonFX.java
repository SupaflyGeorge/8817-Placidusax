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
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/**
 * TalonFX + CANcoder implementation of the intake pivot.
 * On first boot we zero the Talon's position to 0 (wherever the arm is).
 * After that, the Talon tracks continuous mechanism rotations through its gear ratio.
 */
public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX pivotMotor = new TalonFX(Constants.IntakePivotConstants.PIVOT_ID);
  private final CANcoder pivotEncoder = new CANcoder(Constants.IntakePivotConstants.PIVOT_CANCODER_ID);
  private final DutyCycleOut duty = new DutyCycleOut(0).withEnableFOC(true);

  private final StatusSignal<Angle> motorPos = pivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> motorVel = pivotMotor.getVelocity();
  private final StatusSignal<Voltage> motorVolts = pivotMotor.getMotorVoltage();
  private final StatusSignal<Current> motorAmps = pivotMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> motorTemp = pivotMotor.getDeviceTemp();
  private final StatusSignal<Angle> cancoderAbs = pivotEncoder.getAbsolutePosition();

  private double lastPercent = 0.0;
  private boolean seeded = false;

  public IntakePivotIOTalonFX() {
    pivotEncoder.getConfigurator().apply(new CANcoderConfiguration());

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // hold position when idle
    cfg.MotorOutput.Inverted = Constants.IntakePivotConstants.PIVOT_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    // SensorToMechanismRatio makes position read in mechanism rotations, not motor rotations
    cfg.Feedback.SensorToMechanismRatio = Constants.IntakePivotConstants.PIVOT_GEAR_RATIO;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivotConstants.PIVOT_SUPPLY_LIMIT_A;
    pivotMotor.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(50,
        motorPos, motorVel, motorVolts, motorAmps, motorTemp, cancoderAbs);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPos, motorVel, motorVolts, motorAmps, motorTemp, cancoderAbs);

    // Zero position once on boot so all our limits start from 0
    if (!seeded) {
      pivotMotor.setPosition(0);
      seeded = true;
    }

    inputs.motorPercent = lastPercent;
    inputs.motorVolts = motorVolts.getValueAsDouble();
    inputs.motorAmps = motorAmps.getValueAsDouble();
    inputs.motorTempC = motorTemp.getValueAsDouble();
    inputs.pivotPositionRot = motorPos.getValueAsDouble();
    inputs.pivotVelocityRps = motorVel.getValueAsDouble();
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
