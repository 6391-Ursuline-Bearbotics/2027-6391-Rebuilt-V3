package frc.robot;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStation.Alliance;
import java.util.Optional;

/**
 * Utility class for 2026 REBUILT game data. Determines hub active/inactive status based on the game
 * data character ('R' or 'B') and match time shift schedule.
 */
public class GameData {
  private static final double SPINUP_LEAD_TIME_SECONDS = 5.0;
  private static final double FEED_LEAD_TIME_SECONDS = 2.0;

  // Cached parse of the game-specific message ('R'/'B' → true/false). Null means not yet received.
  private static Boolean cachedRedInactiveFirst = null;

  private GameData() {}

  /** Returns true if our alliance's hub is currently active. */
  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return true;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }
    return isHubActiveInTeleop(alliance.get(), DriverStation.getMatchTime());
  }

  /**
   * Returns true if our hub is currently inactive but will become active within the given lead
   * time. Useful for triggering warnings before a shift starts.
   */
  public static boolean isHubActivatingSoon(double leadTimeSeconds) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }
    double matchTime = DriverStation.getMatchTime();
    return !isHubActiveInTeleop(alliance.get(), matchTime)
        && isHubActiveInTeleop(alliance.get(), matchTime - leadTimeSeconds);
  }

  /** Returns true if our hub is active or will be within the spinup lead time. */
  public static boolean isHubActiveOrSoon() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return true;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }
    double matchTime = DriverStation.getMatchTime();
    return isHubActiveInTeleop(alliance.get(), matchTime)
        || isHubActiveInTeleop(alliance.get(), matchTime - SPINUP_LEAD_TIME_SECONDS);
  }

  /**
   * Returns true if the shooter is allowed to spin up at the given position. Always allows spinup
   * outside our alliance zone. Inside our zone, requires hub active or within 5s of becoming
   * active. Only gates when game data is present.
   */
  public static boolean canSpinUp(Translation2d robotPosition) {
    if (!DriverStation.isTeleopEnabled()) {
      return true;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return true;
    }
    boolean isRed = alliance.get() == Alliance.Red;
    if (!FieldConstants.isInOwnAllianceZone(robotPosition, isRed)) {
      return true;
    }
    return isHubActiveOrSoon();
  }

  /**
   * Returns true if the indexer is allowed to feed at the given position. Always allows feeding
   * outside our alliance zone. Inside our zone, requires hub to be active or within the feed lead
   * time of becoming active. Only gates when game data is present.
   */
  public static boolean canFeed(Translation2d robotPosition) {
    if (!DriverStation.isTeleopEnabled()) {
      return true;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return true;
    }
    boolean isRed = alliance.get() == Alliance.Red;
    if (!FieldConstants.isInOwnAllianceZone(robotPosition, isRed)) {
      return true;
    }
    double matchTime = DriverStation.getMatchTime();
    return isHubActiveInTeleop(alliance.get(), matchTime)
        || isHubActiveInTeleop(alliance.get(), matchTime - FEED_LEAD_TIME_SECONDS);
  }

  /**
   * Evaluates hub activity for a given alliance at a specific match time. Returns true (hub active)
   * when no game data is present, during transition/end game periods, and during our active shifts.
   */
  private static boolean isHubActiveInTeleop(Alliance alliance, double matchTime) {
    if (cachedRedInactiveFirst == null) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.isEmpty()) {
        return true; // No game data yet, assume active
      }
      switch (gameData.charAt(0)) {
        case 'R':
          cachedRedInactiveFirst = true;
          break;
        case 'B':
          cachedRedInactiveFirst = false;
          break;
        default:
          return true; // Corrupt data, assume active
      }
    }

    // Shift 1 is active for the alliance that did NOT go inactive first
    boolean shift1Active =
        (alliance == Alliance.Red) ? !cachedRedInactiveFirst : cachedRedInactiveFirst;

    if (matchTime > 130) {
      return true; // Transition shift, always active
    } else if (matchTime > 105) {
      return shift1Active; // Shift 1
    } else if (matchTime > 80) {
      return !shift1Active; // Shift 2
    } else if (matchTime > 55) {
      return shift1Active; // Shift 3
    } else if (matchTime > 30) {
      return !shift1Active; // Shift 4
    } else {
      return true; // End game, always active
    }
  }
}
