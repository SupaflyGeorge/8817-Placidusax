package frc.robot.subsystems.shooterhood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID);
  private final CANcoder hoodEncoder = new CANcoder(Constants.ShooterConstants.HOOD_CANCODER_ID);

  private final PositionVoltage posCtrl = new PositionVoltage(0).withEnableFOC(true);

  private final StatusSignal<Angle> motorPos = hoodMotor.getPosition();
  private final StatusSignal<Voltage> motorVolts = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> motorAmps = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> motorTemp = hoodMotor.getDeviceTemp();

  private final StatusSignal<Angle> cancoderAbs = hoodEncoder.getAbsolutePosition();


  private double targetRot = 0.0;
  private boolean seeded = false;

  public HoodIOTalonFX() {
    // CANcoder basic config
    hoodEncoder.getConfigurator().apply(new CANcoderConfiguration());

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    
    cfg.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.HOOD_GEAR_RATIO;

    cfg.Slot0.kP = Constants.ShooterConstants.HOOD_kP;
    cfg.Slot0.kI = Constants.ShooterConstants.HOOD_kI;
    cfg.Slot0.kD = Constants.ShooterConstants.HOOD_kD;
    cfg.Slot0.kV = Constants.ShooterConstants.HOOD_kV;

    // Soft limits in MECHANISM rotations
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ShooterConstants.HOOD_MAX_ROT;

    
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ShooterConstants.HOOD_MIN_ROT;

    // If hood direction is wrong, flip this one boolean later
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    hoodMotor.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorPos, motorVolts, motorAmps, motorTemp, cancoderAbs
    );
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPos, motorVolts, motorAmps, motorTemp, cancoderAbs);

    if (!seeded) {
      // CANcoder absolute returns rotations (0..1).
     
      System.out.println("motorPos" + hoodEncoder.getPosition());
      hoodEncoder.setPosition(0);
      hoodMotor.setPosition(0);
      seeded = true;
     

    }

    inputs.hoodPositionRot = motorPos.getValueAsDouble();
    inputs.hoodAbsPositionRot = cancoderAbs.getValueAsDouble();
    inputs.hoodTargetRot = targetRot;

    inputs.motorVolts = motorVolts.getValueAsDouble();
    inputs.motorAmps = motorAmps.getValueAsDouble();
    inputs.motorTempC = motorTemp.getValueAsDouble();

    // “homed” here just means we seeded from absolute at least once
    inputs.homed = seeded;
  }

  @Override
  public void setHoodPositionRot(double hoodRot) {
    // clamp to safety
    double clamped = Math.max(Constants.ShooterConstants.HOOD_MIN_ROT, 
                     Math.min(Constants.ShooterConstants.HOOD_MAX_ROT, hoodRot));
    targetRot = clamped;
    hoodMotor.setControl(posCtrl.withPosition(clamped));
  }

  @Override
  public void stop() {
    hoodMotor.stopMotor();
  }
}