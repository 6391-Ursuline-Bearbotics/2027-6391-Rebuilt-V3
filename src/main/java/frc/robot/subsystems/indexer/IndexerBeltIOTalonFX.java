package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class IndexerBeltIOTalonFX implements IndexerBeltIO {
  private final TalonFX talon;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IndexerBeltIOTalonFX() {
    talon = new TalonFX(IndexerConstants.beltMotorId, IndexerConstants.canBus);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        IndexerConstants.beltMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.Slot0 = IndexerConstants.beltGains;
    config.Feedback.SensorToMechanismRatio = IndexerConstants.beltGearRatio;
    config.CurrentLimits.StatorCurrentLimit = IndexerConstants.beltStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    statorCurrent = talon.getStatorCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(IndexerBeltIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    talon.setControl(velocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setGains(double kP, double kV, double kS) {
    var gains = new Slot0Configs().withKP(kP).withKV(kV).withKS(kS);
    tryUntilOk(5, () -> talon.getConfigurator().apply(gains, 0.25));
  }
}
