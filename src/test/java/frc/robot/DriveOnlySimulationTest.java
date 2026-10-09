package frc.robot;

import org.junit.jupiter.api.Test;

class DriveOnlySimulationTest {
  @Test
  void absentMechanismsKeepTeleopAndBothAllianceAutosUsable() throws Exception {
    SimulationChecks.runDriveOnly();
  }
}
