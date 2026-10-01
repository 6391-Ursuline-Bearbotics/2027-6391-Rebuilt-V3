package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.PWMSim;

class ShooterHoodIOServoTest {
  @Test
  void smartIoOutputsUseServoPulseRangeAndClampAngles() {
    assertTrue(HAL.initialize());
    var left = new PWMSim(ShooterConstants.hoodLeftServoPWM);
    var right = new PWMSim(ShooterConstants.hoodRightServoPWM);
    try (var hood = new ShooterHoodIOServo()) {
      assertTrue(left.getInitialized());
      assertTrue(right.getInitialized());
      hood.setAngle(ShooterConstants.hoodMinAngleDeg - 10);
      assertEquals(ShooterConstants.hoodServoMinPulseMicros, left.getPulseMicrosecond());
      assertEquals(left.getPulseMicrosecond(), right.getPulseMicrosecond());
      hood.setAngle((ShooterConstants.hoodMinAngleDeg + ShooterConstants.hoodMaxAngleDeg) / 2);
      assertEquals((ShooterConstants.hoodServoMinPulseMicros
          + ShooterConstants.hoodServoMaxPulseMicros) / 2, left.getPulseMicrosecond());
      hood.setAngle(ShooterConstants.hoodMaxAngleDeg + 10);
      assertEquals(ShooterConstants.hoodServoMaxPulseMicros, left.getPulseMicrosecond());
      assertEquals(left.getPulseMicrosecond(), right.getPulseMicrosecond());
      hood.stop();
      assertEquals(ShooterConstants.hoodServoMinPulseMicros, left.getPulseMicrosecond());
    }
    assertFalse(left.getInitialized());
    assertFalse(right.getInitialized());
  }
}
