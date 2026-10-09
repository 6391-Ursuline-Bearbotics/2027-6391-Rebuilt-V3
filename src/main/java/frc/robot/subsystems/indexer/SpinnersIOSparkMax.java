package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import org.wpilib.hardware.bus.CANPort;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SpinnersIOSparkMax implements SpinnersIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private boolean leftConnected;
  private boolean rightConnected;
  private int currentLimitAmps = IndexerConstants.spinnerCurrentLimitAmps;

  public SpinnersIOSparkMax() {
    leftMotor = new SparkMax(CANPort.CAN_S0, IndexerConstants.leftSpinnerMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(CANPort.CAN_S0, IndexerConstants.rightSpinnerMotorId, MotorType.kBrushless);

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
    // Require a valid response without a CAN fault; cached readings alone are insufficient.
    var leftFaults = leftMotor.getFaults();
    var rightFaults = rightMotor.getFaults();
    boolean leftOk = leftFaults.isValid() && !leftFaults.get().can;
    boolean rightOk = rightFaults.isValid() && !rightFaults.get().can;
    inputs.leftConnected = leftOk;
    inputs.rightConnected = rightOk;

    // Reapply the desired runtime limit once when an optional spinner returns.
    if (inputs.leftConnected && !leftConnected) {
      applyConfig(leftMotor, true, PersistMode.kNoPersistParameters);
    }
    if (inputs.rightConnected && !rightConnected) {
      applyConfig(rightMotor, false, PersistMode.kNoPersistParameters);
    }
    leftConnected = inputs.leftConnected;
    rightConnected = inputs.rightConnected;

    inputs.leftVelocityRPM = leftEncoder.getVelocity().get(0.0);
    inputs.rightVelocityRPM = rightEncoder.getVelocity().get(0.0);
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent().get(0.0);
    inputs.rightCurrentAmps = rightMotor.getOutputCurrent().get(0.0);
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput().get(0.0) * leftMotor.getBusVoltage().get(0.0);
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput().get(0.0) * rightMotor.getBusVoltage().get(0.0);
  }

  @Override
  public void setSpeed(double speed) {
    leftMotor.setThrottle(leftConnected ? speed : 0);
    rightMotor.setThrottle(rightConnected ? speed : 0);
  }

  @Override
  public void stop() {
    leftMotor.setThrottle(0.0);
    rightMotor.setThrottle(0.0);
  }

  @Override
  public void setCurrentLimit(int amps) {
    if (amps == currentLimitAmps) return;
    currentLimitAmps = amps;
    // kNoPersistParameters avoids flash writes on dynamic runtime changes
    if (leftConnected) applyConfig(leftMotor, true, PersistMode.kNoPersistParameters);
    if (rightConnected) applyConfig(rightMotor, false, PersistMode.kNoPersistParameters);
  }
}
