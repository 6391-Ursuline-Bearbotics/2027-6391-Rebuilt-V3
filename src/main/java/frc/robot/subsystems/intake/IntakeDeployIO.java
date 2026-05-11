package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {
  @AutoLog
  public static class IntakeDeployIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakeDeployIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void setBrakeMode(boolean enable) {}
}
