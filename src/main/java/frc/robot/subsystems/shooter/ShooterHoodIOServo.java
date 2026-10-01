package frc.robot.subsystems.shooter;

import org.wpilib.hardware.discrete.PWM;

public class ShooterHoodIOServo implements ShooterHoodIO, AutoCloseable {
  private final PWM leftServo;
  private final PWM rightServo;

  private double commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;

  public ShooterHoodIOServo() {
    leftServo = new PWM(ShooterConstants.hoodLeftServoPWM);
    rightServo = new PWM(ShooterConstants.hoodRightServoPWM);
    leftServo.setOutputPeriod(20);
    rightServo.setOutputPeriod(20);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.positionDeg = commandedAngleDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
    commandedAngleDeg =
        Math.clamp(
            angleDeg, ShooterConstants.hoodMinAngleDeg, ShooterConstants.hoodMaxAngleDeg);
    double servoPosition =
        (commandedAngleDeg - ShooterConstants.hoodMinAngleDeg)
            / (ShooterConstants.hoodMaxAngleDeg - ShooterConstants.hoodMinAngleDeg);
    int pulseMicros = (int) Math.round(
        ShooterConstants.hoodServoMinPulseMicros + servoPosition
            * (ShooterConstants.hoodServoMaxPulseMicros - ShooterConstants.hoodServoMinPulseMicros));
    leftServo.setPulseTimeMicroseconds(pulseMicros);
    rightServo.setPulseTimeMicroseconds(pulseMicros);
  }

  @Override
  public void stop() {
    setAngle(ShooterConstants.hoodMinAngleDeg);
  }

  @Override
  public void close() {
    leftServo.close();
    rightServo.close();
  }
}
