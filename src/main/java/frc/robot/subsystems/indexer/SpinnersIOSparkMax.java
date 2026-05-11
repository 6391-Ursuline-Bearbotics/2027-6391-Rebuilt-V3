package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.math.filter.Debouncer;

public class SpinnersIOSparkMax implements SpinnersIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final Debouncer leftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private int currentLimitAmps = IndexerConstants.spinnerCurrentLimitAmps;

  public SpinnersIOSparkMax() {
    leftMotor = new SparkMax(IndexerConstants.leftSpinnerMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(IndexerConstants.rightSpinnerMotorId, MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    applyConfig(leftMotor, true, PersistMode.kPersistParameters);
    applyConfig(rightMotor, false, PersistMode.kPersistParameters);
  }

  private void applyConfig(SparkMax motor, boolean inverted, PersistMode persistMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(inverted).smartCurrentLimit(currentLimitAmps);
    motor.configure(config, ResetMode.kResetSafeParameters, persistMode);
  }

  @Override
  public void updateInputs(SpinnersIOInputs inputs) {
    // Connection detected by whether the motor responds without faults (no sticky faults = present)
    boolean leftOk = leftMotor.getFaults().can == false;
    boolean rightOk = rightMotor.getFaults().can == false;
    inputs.leftConnected = leftConnectedDebounce.calculate(leftOk);
    inputs.rightConnected = rightConnectedDebounce.calculate(rightOk);

    inputs.leftVelocityRPM = leftEncoder.getVelocity();
    inputs.rightVelocityRPM = rightEncoder.getVelocity();
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
    inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
  }

  @Override
  public void setSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  @Override
  public void stop() {
    leftMotor.set(0.0);
    rightMotor.set(0.0);
  }

  @Override
  public void setCurrentLimit(int amps) {
    if (amps == currentLimitAmps) return;
    currentLimitAmps = amps;
    // kNoPersistParameters avoids flash writes on dynamic runtime changes
    applyConfig(leftMotor, true, PersistMode.kNoPersistParameters);
    applyConfig(rightMotor, false, PersistMode.kNoPersistParameters);
  }
}
