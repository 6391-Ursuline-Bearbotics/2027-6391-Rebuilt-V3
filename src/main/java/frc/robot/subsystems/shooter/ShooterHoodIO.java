package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {
  @AutoLog
  public static class ShooterHoodIOInputs {
    public double positionDeg = 0.0;
  }

  public default void updateInputs(ShooterHoodIOInputs inputs) {}

  /** Sets the hood angle in degrees. Both servos are commanded to the same position. */
  public default void setAngle(double angleDeg) {}

  public default void stop() {}
}
