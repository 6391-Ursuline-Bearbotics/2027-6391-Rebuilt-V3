package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;

class RobotStartupTest {
  @Test
  void simulationConstructsAndRunsDisabledSchedulerCycles() {
    assertTrue(HAL.initialize());
    assertDoesNotThrow(() -> {
      try (var robot = new Robot()) {
        robot.simulationInit();
        robot.disabledInit();
        for (int cycle = 0; cycle < 10; cycle++) {
          robot.robotPeriodic();
          robot.disabledPeriodic();
          robot.simulationPeriodic();
        }
      } finally {
        Scheduler.getDefault().cancelAll();
      }
    });
  }
}
