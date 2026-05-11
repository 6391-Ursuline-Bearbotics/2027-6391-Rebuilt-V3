package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpinnersIO {
  @AutoLog
  public static class SpinnersIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double leftVelocityRPM = 0.0;
    public double rightVelocityRPM = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
  }

  public default void updateInputs(SpinnersIOInputs inputs) {}

  /** Set both spinners to the same duty cycle speed (-1.0 to 1.0). */
  public default void setSpeed(double speed) {}

  public default void stop() {}

  public default void setCurrentLimit(int amps) {}
}
