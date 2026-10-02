package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.junit.jupiter.api.Test;

class ShooterSimulationTest {
  @Test
  void phoenixRotationGainsAreConvertedForRadiansSimulation() {
    assertTrue(org.wpilib.hardware.hal.HAL.initialize());
    var sim = new ShooterIOSim();
    var inputs = new ShooterIO.ShooterIOInputs();
    sim.setGains(0.1, 0.12, 0);
    double target = 3000 * 2 * Math.PI / 60;
    for (int cycle = 0; cycle < 250; cycle++) {
      sim.setVelocityFOC(target); // Hybrid SHOOT starts with this request, never a no-op.
      sim.updateInputs(inputs);
    }
    assertEquals(target, inputs.leftVelocityRadPerSec, 100 * 2 * Math.PI / 60);
    assertEquals(target, inputs.rightVelocityRadPerSec, 100 * 2 * Math.PI / 60);
    sim.stop();
    for (int cycle = 0; cycle < 100; cycle++) sim.updateInputs(inputs);
    assertEquals(0, inputs.leftVelocityRadPerSec, 0.1);
  }
}
