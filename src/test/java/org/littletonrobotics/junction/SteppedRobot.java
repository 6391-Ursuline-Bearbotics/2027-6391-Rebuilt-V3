package org.littletonrobotics.junction;

import frc.robot.Robot;

/** Test-only access to the same AdvantageKit hooks used by LoggedRobot's main loop. */
public final class SteppedRobot extends Robot {
  public void step() {
    Logger.periodicBeforeUser();
    try {
      loopFunc();
    } finally {
      Logger.periodicAfterUser(0, 0);
    }
  }
}
