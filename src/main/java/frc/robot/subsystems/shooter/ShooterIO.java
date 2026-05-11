package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double leftVelocityRadPerSec = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftStatorCurrentAmps = 0.0;
    public double rightStatorCurrentAmps = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;
    public double rightTempCelsius = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets both motors to the same velocity setpoint. */
  public default void setVelocity(double velocityRadPerSec) {}

  public default void setVoltage(double volts) {}

  /** Sets both motors to a velocity setpoint using VelocityTorqueCurrentFOC (bang-bang mode). */
  public default void setVelocityFOC(double velocityRadPerSec) {}

  public default void stop() {}

  public default void setGains(double kP, double kV, double kS) {}

  /** Sets the peak torque current limit for FOC bang-bang mode. */
  public default void setPeakTorqueCurrent(double amps) {}
}
