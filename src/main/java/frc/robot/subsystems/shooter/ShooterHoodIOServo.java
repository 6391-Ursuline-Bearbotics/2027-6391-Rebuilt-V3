package frc.robot.subsystems.shooter;

import org.wpilib.math.util.MathUtil;
import org.wpilib.hardware.servo.Servo;

public class ShooterHoodIOServo implements ShooterHoodIO {
  private final Servo leftServo;
  private final Servo rightServo;

  private double commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;

  public ShooterHoodIOServo() {
    leftServo = new Servo(ShooterConstants.hoodLeftServoPWM);
    rightServo = new Servo(ShooterConstants.hoodRightServoPWM);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.positionDeg = commandedAngleDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
    commandedAngleDeg =
        MathUtil.clamp(
            angleDeg, ShooterConstants.hoodMinAngleDeg, ShooterConstants.hoodMaxAngleDeg);
    double servoPosition =
        (commandedAngleDeg - ShooterConstants.hoodMinAngleDeg)
            / (ShooterConstants.hoodMaxAngleDeg - ShooterConstants.hoodMinAngleDeg);
    leftServo.set(servoPosition);
    rightServo.set(servoPosition);
  }

  @Override
  public void stop() {
    setAngle(ShooterConstants.hoodMinAngleDeg);
  }
}
