package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC velocityFOCRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);

  // Left motor signals
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Temperature> leftTemp;

  // Right motor signals
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightStatorCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Temperature> rightTemp;

  private final Debouncer leftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer rightConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public ShooterIOTalonFX() {
    leftTalon = new TalonFX(ShooterConstants.leftMotorId, ShooterConstants.canBus);
    rightTalon = new TalonFX(ShooterConstants.rightMotorId, ShooterConstants.canBus);

    // Configure left motor
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.MotorOutput.Inverted =
        ShooterConstants.leftMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftConfig.Slot0 = ShooterConstants.gains;
    leftConfig.Slot1.kP = ShooterConstants.bangBangKp;
    leftConfig.Feedback.SensorToMechanismRatio = ShooterConstants.gearRatio;
    leftConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorCurrentLimit;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.bangBangPeakCurrentAmps;
    leftConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    leftConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    leftConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftConfig, 0.25));

    // Configure right motor
    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.Inverted =
        ShooterConstants.rightMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    rightConfig.Slot0 = ShooterConstants.gains;
    rightConfig.Slot1.kP = ShooterConstants.bangBangKp;
    rightConfig.Feedback.SensorToMechanismRatio = ShooterConstants.gearRatio;
    rightConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorCurrentLimit;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.bangBangPeakCurrentAmps;
    rightConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    rightConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    rightConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
    tryUntilOk(5, () -> rightTalon.getConfigurator().apply(rightConfig, 0.25));

    // Get status signals
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftStatorCurrent = leftTalon.getStatorCurrent();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTemp = leftTalon.getDeviceTemp();

    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightStatorCurrent = rightTalon.getStatorCurrent();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTemp = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftAppliedVolts,
        leftStatorCurrent,
        leftSupplyCurrent,
        leftTemp,
        rightVelocity,
        rightAppliedVolts,
        rightStatorCurrent,
        rightSupplyCurrent,
        rightTemp);
    ParentDevice.optimizeBusUtilizationForAll(leftTalon, rightTalon);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var leftStatus =
        BaseStatusSignal.refreshAll(
            leftVelocity, leftAppliedVolts, leftStatorCurrent, leftSupplyCurrent, leftTemp);
    var rightStatus =
        BaseStatusSignal.refreshAll(
            rightVelocity, rightAppliedVolts, rightStatorCurrent, rightSupplyCurrent, rightTemp);

    inputs.leftConnected = leftConnectedDebounce.calculate(leftStatus.isOK());
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTemp.getValueAsDouble();

    inputs.rightConnected = rightConnectedDebounce.calculate(rightStatus.isOK());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTemp.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    leftTalon.setControl(velocityRequest.withVelocity(velocityRotPerSec));
    rightTalon.setControl(velocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setVelocityFOC(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    leftTalon.setControl(velocityFOCRequest.withVelocity(velocityRotPerSec));
    rightTalon.setControl(velocityFOCRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setVoltage(double volts) {
    leftTalon.setControl(voltageRequest.withOutput(volts));
    rightTalon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leftTalon.setControl(voltageRequest.withOutput(0.0));
    rightTalon.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setGains(double kP, double kV, double kS) {
    var gains = new Slot0Configs().withKP(kP).withKV(kV).withKS(kS);
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(gains, 0.25));
    tryUntilOk(5, () -> rightTalon.getConfigurator().apply(gains, 0.25));
  }

  @Override
  public void setPeakTorqueCurrent(double amps) {
    var config =
        new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(amps)
            .withPeakReverseTorqueCurrent(0.0);
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> rightTalon.getConfigurator().apply(config, 0.25));
  }
}
