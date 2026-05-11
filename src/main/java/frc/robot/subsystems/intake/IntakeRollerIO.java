package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void setVelocity(double velocityRadPerSec) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void setGains(double kP, double kV, double kS) {}
}
