package frc.robot;

import org.junit.jupiter.api.Test;

class AutonomousCoroutineTest {
  @Test
  void shootingGatheringAndMovingShotScopesCancelOnBothAlliances() throws Exception {
    SimulationChecks.runCoroutinePhases();
  }
}
