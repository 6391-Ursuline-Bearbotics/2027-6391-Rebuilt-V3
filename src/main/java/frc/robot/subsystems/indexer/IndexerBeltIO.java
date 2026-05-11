package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerBeltIO {
  @AutoLog
  public static class IndexerBeltIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IndexerBeltIOInputs inputs) {}

  public default void setVelocity(double velocityRadPerSec) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void setGains(double kP, double kV, double kS) {}
}
