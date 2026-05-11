package frc.robot.subsystems.indexer;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.AlertType;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.GameData;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends Mechanism {
  // Tunable spinner parameters
  private static final LoggedTunableNumber spinnerSpeed =
      new LoggedTunableNumber("Indexer/Spinner/Speed", IndexerConstants.spinnerSpeed);
  private static final LoggedTunableNumber spinnerDelay =
      new LoggedTunableNumber("Indexer/Spinner/DelaySeconds", IndexerConstants.spinnerDelaySeconds);
  private static final LoggedTunableNumber spinnerStallCurrent =
      new LoggedTunableNumber(
          "Indexer/Spinner/StallCurrentAmps", IndexerConstants.spinnerStallCurrentAmps);
  private static final LoggedTunableNumber spinnerCooldown =
      new LoggedTunableNumber(
          "Indexer/Spinner/CooldownSeconds", IndexerConstants.spinnerCooldownSeconds);
  private static final LoggedTunableNumber spinnerCurrentLimit =
      new LoggedTunableNumber(
          "Indexer/Spinner/CurrentLimitAmps", IndexerConstants.spinnerCurrentLimitAmps);
  private static final LoggedTunableNumber spinnerInrushCurrent =
      new LoggedTunableNumber(
          "Indexer/Spinner/InrushCurrentAmps", IndexerConstants.spinnerInrushCurrentAmps);
  private static final LoggedTunableNumber spinnerInrushTime =
      new LoggedTunableNumber(
          "Indexer/Spinner/InrushTimeSeconds", IndexerConstants.spinnerInrushTimeSeconds);

  // Tunable belt gains
  private static final LoggedTunableNumber beltKp = new LoggedTunableNumber("Indexer/Belt/kP", 0.1);
  private static final LoggedTunableNumber beltKv =
      new LoggedTunableNumber("Indexer/Belt/kV", 0.12);
  private static final LoggedTunableNumber beltKs = new LoggedTunableNumber("Indexer/Belt/kS", 0.0);

  // Tunable kicker gains
  private static final LoggedTunableNumber kickerKp =
      new LoggedTunableNumber("Indexer/Kicker/kP", 0.1);
  private static final LoggedTunableNumber kickerKv =
      new LoggedTunableNumber("Indexer/Kicker/kV", 0.12);
  private static final LoggedTunableNumber kickerKs =
      new LoggedTunableNumber("Indexer/Kicker/kS", 0.0);

  // Tunable jam detection parameters
  private static final LoggedTunableNumber jamCurrentThreshold =
      new LoggedTunableNumber("Indexer/JamCurrentThreshold", IndexerConstants.jamCurrentThreshold);
  private static final LoggedTunableNumber jamDebounceTime =
      new LoggedTunableNumber("Indexer/JamDebounceTime", IndexerConstants.jamDebounceTime);
  private static final LoggedTunableNumber jamReverseTime =
      new LoggedTunableNumber("Indexer/JamReverseTime", IndexerConstants.jamReverseTime);

  // Tunable velocity setpoints
  private static final LoggedTunableNumber beltFeedRPM =
      new LoggedTunableNumber("Indexer/Belt/FeedRPM", IndexerConstants.beltFeedVelocityRPM);
  private static final LoggedTunableNumber beltEjectRPM =
      new LoggedTunableNumber("Indexer/Belt/EjectRPM", IndexerConstants.beltEjectVelocityRPM);
  private static final LoggedTunableNumber kickerFeedRPM =
      new LoggedTunableNumber("Indexer/Kicker/FeedRPM", IndexerConstants.kickerFeedVelocityRPM);
  private static final LoggedTunableNumber kickerEjectRPM =
      new LoggedTunableNumber("Indexer/Kicker/EjectRPM", IndexerConstants.kickerEjectVelocityRPM);

  public enum Goal {
    IDLE,
    FEED,
    EJECT
  }

  // IO layers
  private final IndexerBeltIO beltIO;
  private final IndexerBeltIOInputsAutoLogged beltInputs = new IndexerBeltIOInputsAutoLogged();
  private final IndexerKickerIO kickerIO;
  private final IndexerKickerIOInputsAutoLogged kickerInputs =
      new IndexerKickerIOInputsAutoLogged();
  private final SpinnersIO spinnersIO;
  private final SpinnersIOInputsAutoLogged spinnersInputs = new SpinnersIOInputsAutoLogged();

  // Pose supplier for game data gating
  private final Supplier<Pose2d> poseSupplier;

  // State
  private Goal goal = Goal.IDLE;
  private boolean feedUngated = false;

  // Jam detection
  private final Timer jamTimer = new Timer();
  private final Timer jamReverseTimer = new Timer();
  private boolean jammed = false;
  private Goal goalBeforeJam = Goal.IDLE;

  // Spinner state
  private final Timer spinnerDelayTimer = new Timer();
  private boolean spinnerDelayStarted = false;
  private final Timer spinnerCooldownTimer = new Timer();
  private boolean spinnerInCooldown = false;
  private boolean spinnerNeedsInrush = true; // true whenever motors were last stopped
  private boolean spinnerInrushActive = false;
  private final Timer spinnerInrushTimer = new Timer();

  // Alerts
  private final Alert beltDisconnectedAlert =
      new Alert("Indexer belt motor disconnected.", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Indexer kicker motor disconnected.", AlertType.kError);
  private final Alert beltOverTempAlert =
      new Alert("Indexer belt motor over temperature.", AlertType.kWarning);
  private final Alert kickerOverTempAlert =
      new Alert("Indexer kicker motor over temperature.", AlertType.kWarning);
  private final Alert jamAlert = new Alert("Indexer jam detected.", AlertType.kWarning);
  private final Alert leftSpinnerDisconnectedAlert =
      new Alert("Left spinner motor disconnected.", AlertType.kError);
  private final Alert rightSpinnerDisconnectedAlert =
      new Alert("Right spinner motor disconnected.", AlertType.kError);

  public Indexer(
      IndexerBeltIO beltIO,
      IndexerKickerIO kickerIO,
      SpinnersIO spinnersIO,
      Supplier<Pose2d> poseSupplier) {
    this.beltIO = beltIO;
    this.kickerIO = kickerIO;
    this.spinnersIO = spinnersIO;
    this.poseSupplier = poseSupplier;
    jamTimer.start();
    spinnersIO.setCurrentLimit(IndexerConstants.spinnerCurrentLimitAmps);
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @AutoLogOutput(key = "Indexer/Goal")
  public Goal getGoal() {
    return goal;
  }

  public void periodic() {
    // Update and log inputs
    beltIO.updateInputs(beltInputs);
    Logger.processInputs("Indexer/Belt", beltInputs);
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);
    spinnersIO.updateInputs(spinnersInputs);
    Logger.processInputs("Indexer/Spinners", spinnersInputs);

    // Stop everything when disabled and reset spinner state so it re-initializes cleanly on enable
    if (DriverStation.isDisabled()) {
      beltIO.stop();
      kickerIO.stop();
      spinnersIO.stop();
      spinnerDelayStarted = false;
      spinnerInCooldown = false;
      spinnerNeedsInrush = true;
      spinnerInrushActive = false;
      updateAlerts();
      return;
    }

    // Update belt gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> beltIO.setGains(values[0], values[1], values[2]),
        beltKp,
        beltKv,
        beltKs);

    // Update kicker gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        values -> kickerIO.setGains(values[0], values[1], values[2]),
        kickerKp,
        kickerKv,
        kickerKs);

    // Determine commanded velocities and whether motors are active
    double commandedBeltVel = 0.0;
    double commandedKickerVel = 0.0;
    boolean motorsActive = false;

    switch (goal) {
      case FEED:
        if (feedUngated || GameData.canFeed(poseSupplier.get().getTranslation())) {
          commandedBeltVel = rpmToRadPerSec(beltFeedRPM.get());
          commandedKickerVel = rpmToRadPerSec(kickerFeedRPM.get());
          motorsActive = true;
        }
        break;
      case EJECT:
        commandedBeltVel = rpmToRadPerSec(beltEjectRPM.get());
        commandedKickerVel = rpmToRadPerSec(kickerEjectRPM.get());
        motorsActive = true;
        break;
      default:
        break;
    }

    // Jam detection: either belt or kicker stalled while motors are active
    boolean beltStall =
        motorsActive
            && beltInputs.statorCurrentAmps > jamCurrentThreshold.get()
            && Math.abs(beltInputs.velocityRadPerSec) < Math.abs(commandedBeltVel) * 0.1;
    boolean kickerStall =
        motorsActive
            && kickerInputs.statorCurrentAmps > jamCurrentThreshold.get()
            && Math.abs(kickerInputs.velocityRadPerSec) < Math.abs(commandedKickerVel) * 0.1;

    if (!beltStall && !kickerStall) {
      jamTimer.restart();
    }

    // Handle jam state
    if (jammed) {
      // Reversing both motors to clear jam — stop spinners to avoid conflicting ball flow
      beltIO.setVelocity(rpmToRadPerSec(beltEjectRPM.get()));
      kickerIO.setVelocity(rpmToRadPerSec(kickerEjectRPM.get()));
      spinnersIO.stop();
      spinnerDelayStarted = false;
      spinnerNeedsInrush = true;
      spinnerInrushActive = false;
      if (jamReverseTimer.hasElapsed(jamReverseTime.get())) {
        // Done reversing, resume previous goal
        jammed = false;
        goal = goalBeforeJam;
      }
    } else if (motorsActive && jamTimer.hasElapsed(jamDebounceTime.get())) {
      // Jam detected — start reversing both motors
      jammed = true;
      goalBeforeJam = goal;
      jamReverseTimer.restart();
      beltIO.setVelocity(rpmToRadPerSec(beltEjectRPM.get()));
      kickerIO.setVelocity(rpmToRadPerSec(kickerEjectRPM.get()));
    } else if (motorsActive) {
      beltIO.setVelocity(commandedBeltVel);
      kickerIO.setVelocity(commandedKickerVel);
    } else {
      beltIO.stop();
      kickerIO.stop();
      jammed = false;
    }

    // Update spinner current limit if tuned
    LoggedTunableNumber.ifChanged(
        hashCode() + 2, values -> spinnersIO.setCurrentLimit((int) values[0]), spinnerCurrentLimit);

    // Spinner logic: turn on 0.5s after kicker/belt, with stall auto-recovery
    if (motorsActive) {
      if (!spinnerDelayStarted) {
        spinnerDelayTimer.restart();
        spinnerDelayStarted = true;
      }
      if (spinnerDelayTimer.hasElapsed(spinnerDelay.get())) {
        // Suppress stall detection during startup (before and throughout inrush window) so normal
        // spin-up current + low velocity does not false-trip into cooldown
        boolean inStartup = spinnerNeedsInrush || spinnerInrushActive;
        boolean leftStall =
            !inStartup
                && spinnersInputs.leftCurrentAmps > spinnerStallCurrent.get()
                && Math.abs(spinnersInputs.leftVelocityRPM) < 11000.0 * spinnerSpeed.get() * 0.1;
        boolean rightStall =
            !inStartup
                && spinnersInputs.rightCurrentAmps > spinnerStallCurrent.get()
                && Math.abs(spinnersInputs.rightVelocityRPM) < 11000.0 * spinnerSpeed.get() * 0.1;

        if (spinnerInCooldown) {
          spinnersIO.stop();
          if (spinnerCooldownTimer.hasElapsed(spinnerCooldown.get())) {
            spinnerInCooldown = false;
          }
        } else if (leftStall || rightStall) {
          spinnerInCooldown = true;
          spinnerCooldownTimer.restart();
          spinnerNeedsInrush = true;
          spinnerInrushActive = false;
          spinnersIO.stop();
        } else {
          // Apply inrush current on the first cycle after each stop, then revert to normal limit
          if (spinnerNeedsInrush) {
            spinnersIO.setCurrentLimit((int) spinnerInrushCurrent.get());
            spinnerInrushTimer.restart();
            spinnerNeedsInrush = false;
            spinnerInrushActive = true;
          } else if (spinnerInrushActive
              && spinnerInrushTimer.hasElapsed(spinnerInrushTime.get())) {
            spinnersIO.setCurrentLimit((int) spinnerCurrentLimit.get());
            spinnerInrushActive = false;
          }
          spinnersIO.setSpeed(spinnerSpeed.get());
        }
      }
      // else still in delay window — leave spinners stopped
    } else {
      spinnersIO.stop();
      spinnerDelayStarted = false;
      spinnerInCooldown = false;
      spinnerNeedsInrush = true;
      spinnerInrushActive = false;
    }

    updateAlerts();
  }

  @AutoLogOutput(key = "Indexer/Jammed")
  public boolean isJammed() {
    return jammed;
  }

  private void updateAlerts() {
    beltDisconnectedAlert.set(!beltInputs.connected && Constants.currentMode != Mode.SIM);
    kickerDisconnectedAlert.set(!kickerInputs.connected && Constants.currentMode != Mode.SIM);
    beltOverTempAlert.set(beltInputs.tempCelsius > 80.0);
    kickerOverTempAlert.set(kickerInputs.tempCelsius > 80.0);
    jamAlert.set(jammed);
    leftSpinnerDisconnectedAlert.set(
        !spinnersInputs.leftConnected && Constants.currentMode != Mode.SIM);
    rightSpinnerDisconnectedAlert.set(
        !spinnersInputs.rightConnected && Constants.currentMode != Mode.SIM);
  }

  private static double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  // Command factories
  public Command setGoalCommand(Goal goal) {
    return Commands.runOnce(() -> setGoal(goal)).withName("Indexer " + goal.name());
  }

  public Command feedCommand() {
    return Commands.startEnd(() -> setGoal(Goal.FEED), () -> setGoal(Goal.IDLE), this)
        .withName("Indexer Feed");
  }

  public Command feedUngatedCommand() {
    return Commands.startEnd(
            () -> {
              feedUngated = true;
              setGoal(Goal.FEED);
            },
            () -> {
              feedUngated = false;
              setGoal(Goal.IDLE);
            },
            this)
        .withName("Indexer Feed Ungated");
  }

  public Command ejectCommand() {
    return Commands.startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE), this)
        .withName("Indexer Eject");
  }
}
