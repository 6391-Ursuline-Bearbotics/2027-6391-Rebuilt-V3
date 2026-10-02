package frc.robot;

import org.junit.jupiter.api.Test;

class SimulationFunctionalTest {
  @Test
  void teleopAndFourAutosOnBothAlliances() throws Exception {
    SimulationChecks.run(false);
  }
}
