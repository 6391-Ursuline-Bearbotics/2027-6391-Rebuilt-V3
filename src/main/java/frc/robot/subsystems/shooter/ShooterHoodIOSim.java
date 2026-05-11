package frc.robot.subsystems.shooter;

import org.wpilib.math.util.MathUtil;

public class ShooterHoodIOSim implements ShooterHoodIO {
  private double positionDeg = ShooterConstants.hoodMinAngleDeg;

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.positionDeg = positionDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
    positionDeg =
        MathUtil.clamp(
            angleDeg, ShooterConstants.hoodMinAngleDeg, ShooterConstants.hoodMaxAngleDeg);
  }

  @Override
  public void stop() {
    positionDeg = ShooterConstants.hoodMinAngleDeg;
  }
}
