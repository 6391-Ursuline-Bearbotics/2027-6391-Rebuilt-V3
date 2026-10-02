package frc.robot.subsystems.intake;

import frc.robot.util.V3Commands;

import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake implements Mechanism {
  // Tunable roller gains
  private static final LoggedTunableNumber rollerKp =
      new LoggedTunableNumber("Intake/Roller/kP", 0.6);
  private static final LoggedTunableNumber rollerKv =
      new LoggedTunableNumber("Intake/Roller/kV", 0.119);
  private static final LoggedTunableNumber rollerKs =
      new LoggedTunableNumber("Intake/Roller/kS", 0.24691);

  // Tunable roller velocity setpoints
  private static final LoggedTunableNumber rollerIntakeRPM =
      new LoggedTunableNumber("Intake/Roller/IntakeRPM", IntakeConstants.rollerIntakeVelocityRPM);
  private static final LoggedTunableNumber rollerClumpIntakeRPM =
      new LoggedTunableNumber(
          "Intake/Roller/ClumpIntakeRPM", IntakeConstants.rollerClumpIntakeVelocityRPM);
  private static final LoggedTunableNumber rollerEjectRPM =
      new LoggedTunableNumber("Intake/Roller/EjectRPM", IntakeConstants.rollerEjectVelocityRPM);

  // Tunable deploy parameters
  private static final LoggedTunableNumber deployVoltage =
      new LoggedTunableNumber("Intake/Deploy/Voltage", IntakeConstants.deployVoltage);
  private static final LoggedTunableNumber retractVoltage =
      new LoggedTunableNumber("Intake/Deploy/RetractVoltage", IntakeConstants.retractVoltage);
  private static final LoggedTunableNumber stallCurrentThreshold =
      new LoggedTunableNumber(
          "Intake/Deploy/StallCurrentThreshold", IntakeConstants.deployCurrentThreshold);
  private static final LoggedTunableNumber retractStallCurrentThreshold =
      new LoggedTunableNumber(
          "Intake/Deploy/RetractStallCurrentThreshold", IntakeConstants.retractCurrentThreshold);
  private static final LoggedTunableNumber retractRollerShutoffFraction =
      new LoggedTunableNumber("Intake/Deploy/RetractRollerShutoffFraction", 0.9);
  private static final LoggedTunableNumber rehomeDeployTime =
      new LoggedTunableNumber("Intake/Deploy/RehomeDeployTime", 0.5);

  // Light voltage to hold intake retracted against gravity (only applied when drifted from hard
  // stop)
  private static final LoggedTunableNumber retractHoldVoltage =
      new LoggedTunableNumber("Intake/Deploy/RetractHoldVoltage", -0.4);
  private static final LoggedTunableNumber retractHoldDriftThresholdRad =
      new LoggedTunableNumber("Intake/Deploy/RetractHoldDriftThresholdRad", 0.05);

  // Position hold: resist light forces when deployed, yield to heavy forces
  private static final LoggedTunableNumber positionHoldThresholdRad =
      new LoggedTunableNumber("Intake/Deploy/PositionHoldThresholdRad", 0.15);
  private static final LoggedTunableNumber positionHoldVoltage =
      new LoggedTunableNumber("Intake/Deploy/PositionHoldVoltage", 0);
  private static final LoggedTunableNumber positionHoldYieldCurrentAmps =
      new LoggedTunableNumber("Intake/Deploy/PositionHoldYieldCurrentAmps", 30.0);
  private static final LoggedTunableNumber positionHoldTimeoutSecs =
      new LoggedTunableNumber("Intake/Deploy/PositionHoldTimeoutSecs", 0.15);

  // Tunable roller jam detection parameters
  private static final LoggedTunableNumber rollerJamCurrentThreshold =
      new LoggedTunableNumber(
          "Intake/Roller/JamCurrentThreshold", IntakeConstants.rollerJamCurrentThreshold);
  private static final LoggedTunableNumber rollerJamDebounceTime =
      new LoggedTunableNumber(
          "Intake/Roller/JamDebounceTime", IntakeConstants.rollerJamDebounceTime);

  /** Desired intake behavior, set by operator controls. */
  public enum Goal {
    IDLE, // Retracted, rollers off
    INTAKE, // Deployed, rollers spinning inward
    EJECT, // Deployed, rollers spinning outward
    DEPLOYED_IDLE, // Deployed, rollers off
    CLUMP_INTAKE // Deployed, rollers at higher speed for clumps
  }

  /** Internal deploy mechanism state. */
  private enum DeployState {
    RETRACTED,
    DEPLOYING,
    DEPLOYED,
    RETRACTING,
    REHOME_DEPLOYING // Brief deploy bump to reseat when already retracted
  }

  // IO layers
  private final IntakeDeployIO deployIO;
  private final IntakeDeployIOInputsAutoLogged deployInputs = new IntakeDeployIOInputsAutoLogged();
  private final IntakeRollerIO rollerIO;
  private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

  // State
  private Goal goal = Goal.IDLE;
  private DeployState deployState = DeployState.RETRACTED;
  private boolean shootingPressureMode = false;
  private double deployedPositionRad = 0.0;
  private double retractedPositionRad = 0.0;
  private boolean rehomeRequested = false;
  // Latches true when pushed back past threshold; stays true until fully returned to deployed
  // position, preventing bang-bang oscillation.
  private boolean positionCorrectionActive = false;
  // Set when correction times out; blocks re-arming until intake returns to setpoint.
  private boolean positionCorrectionGaveUp = false;

  // Deploy current spike detection
  private final Timer stallTimer = new Timer();

  // Position correction timeout
  private final Timer positionCorrectionTimer = new Timer();

  // Roller jam detection
  private final Timer rollerJamTimer = new Timer();
  private boolean rollerJammed = false;

  // Alerts
  private final Alert deployDisconnectedAlert =
      new Alert("Intake deploy motor disconnected.", "Intake deploy motor disconnected.", Level.HIGH);
  private final Alert rollerDisconnectedAlert =
      new Alert("Intake roller motor disconnected.", "Intake roller motor disconnected.", Level.HIGH);
  private final Alert deployOverTempAlert =
      new Alert("Intake deploy motor over temperature.", "Intake deploy motor over temperature.", Level.MEDIUM);
  private final Alert rollerOverTempAlert =
      new Alert("Intake roller motor over temperature.", "Intake roller motor over temperature.", Level.MEDIUM);
  private final Alert rollerJamAlert = new Alert("Intake roller jam detected.", "Intake roller jam detected.", Level.MEDIUM);

  public Intake(IntakeDeployIO deployIO, IntakeRollerIO rollerIO) {
    this.deployIO = deployIO;
    this.rollerIO = rollerIO;
    stallTimer.start();
    rollerJamTimer.start();
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void setGoal(Goal goal) {
    if (goal == Goal.IDLE && deployState == DeployState.RETRACTED) {
      rehomeRequested = true;
    }
    // Re-pressing an active roller goal clears a jam so the operator doesn't need to retract first
    if (goal == Goal.INTAKE || goal == Goal.CLUMP_INTAKE || goal == Goal.EJECT) {
      rollerJammed = false;
    }
    this.goal = goal;
  }

  @AutoLogOutput(key = "Intake/Goal")
  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput(key = "Intake/ShootingPressureMode")
  public boolean isShootingPressureMode() {
    return shootingPressureMode;
  }

  /** Enables or disables shooting pressure mode (continuous retraction voltage at hard stop). */
  public void setShootingPressureMode(boolean enable) {
    shootingPressureMode = enable;
  }

  /**
   * Applies continuous retraction pressure while active, keeping the intake pressed against the
   * indexer during shooting. Does not require the intake subsystem.
   */
  public Command shootingPressureCommand() {
    return V3Commands.startEnd(
        () -> setShootingPressureMode(true), () -> setShootingPressureMode(false));
  }

  @AutoLogOutput(key = "Intake/DeployState")
  public DeployState getDeployState() {
    return deployState;
  }

  @AutoLogOutput(key = "Intake/Deployed")
  public boolean isDeployed() {
    return deployState == DeployState.DEPLOYED;
  }

  @AutoLogOutput(key = "Intake/Retracted")
  public boolean isRetracted() {
    return deployState == DeployState.RETRACTED;
  }

  @AutoLogOutput(key = "Intake/RollerVelocityRPM")
  public double getRollerVelocityRPM() {
    return rollerInputs.velocityRadPerSec * 60.0 / (2.0 * Math.PI);
  }

  /** Run roller at raw voltage for characterization. */
  public void runRollerCharacterization(double volts) {
    rollerIO.setVoltage(volts);
  }

  /** Get roller velocity in rad/s for characterization. */
  public double getRollerCharacterizationVelocity() {
    return rollerInputs.velocityRadPerSec;
  }

  public void periodic() {
    // Update and log inputs
    deployIO.updateInputs(deployInputs);
    Logger.processInputs("Intake/Deploy", deployInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    // Stop everything when disabled
    if (RobotState.isDisabled()) {
      deployIO.stop();
      rollerIO.stop();
      updateAlerts();
      return;
    }

    // Determine if goal wants the intake deployed
    boolean goalWantsDeploy =
        goal == Goal.INTAKE
            || goal == Goal.EJECT
            || goal == Goal.DEPLOYED_IDLE
            || goal == Goal.CLUMP_INTAKE;

    // Current spike detection: ignore inrush, then check threshold
    boolean stallDetected =
        stallTimer.hasElapsed(IntakeConstants.deployInrushIgnoreTime)
            && deployInputs.statorCurrentAmps > stallCurrentThreshold.get();

    // Deploy state machine
    switch (deployState) {
      case RETRACTED:
        if (goalWantsDeploy) {
          rehomeRequested = false;
          transitionTo(DeployState.DEPLOYING);
          deployIO.setVoltage(deployVoltage.get());
        } else if (rehomeRequested) {
          rehomeRequested = false;
          transitionTo(DeployState.REHOME_DEPLOYING);
          deployIO.setVoltage(deployVoltage.get());
        } else if (shootingPressureMode) {
          deployIO.setVoltage(retractVoltage.get());
        } else {
          // Only hold when drifted away from hard stop — avoids fighting it when already seated
          double driftRad = deployInputs.positionRad - retractedPositionRad;
          if (driftRad > retractHoldDriftThresholdRad.get()) {
            deployIO.setVoltage(retractHoldVoltage.get());
          } else {
            deployIO.stop();
          }
        }
        break;

      case DEPLOYING:
        if (!goalWantsDeploy) {
          transitionTo(DeployState.RETRACTING);
          deployIO.setVoltage(retractVoltage.get());
        } else if (stallDetected) {
          transitionTo(DeployState.DEPLOYED);
          deployIO.stop();
        } else {
          deployIO.setVoltage(deployVoltage.get());
        }
        break;

      case DEPLOYED:
        if (!goalWantsDeploy) {
          positionCorrectionActive = false;
          positionCorrectionGaveUp = false;
          transitionTo(DeployState.RETRACTING);
          deployIO.setVoltage(retractVoltage.get());
        } else {
          // Latch correction on once pushed back past threshold; only clear it once fully
          // returned to the deployed position. This prevents bang-bang oscillation.
          double pushBackRad = deployedPositionRad - deployInputs.positionRad;
          if (pushBackRad <= 0.0) {
            // Returned to setpoint — clear all correction state
            positionCorrectionActive = false;
            positionCorrectionGaveUp = false;
          } else if (pushBackRad > positionHoldThresholdRad.get()
              && !positionCorrectionActive
              && !positionCorrectionGaveUp) {
            positionCorrectionActive = true;
            positionCorrectionTimer.restart();
          }

          if (positionCorrectionActive) {
            if (positionCorrectionTimer.hasElapsed(positionHoldTimeoutSecs.get())) {
              // Timed out — give up and block re-arming until intake returns to setpoint
              positionCorrectionActive = false;
              positionCorrectionGaveUp = true;
              deployIO.stop();
            } else if (deployInputs.statorCurrentAmps < positionHoldYieldCurrentAmps.get()) {
              deployIO.setVoltage(positionHoldVoltage.get());
            } else {
              // Heavy force — yield rather than fight it and reset latch
              positionCorrectionActive = false;
              deployIO.stop();
            }
          } else {
            deployIO.stop();
          }
        }
        break;

      case RETRACTING:
        if (goalWantsDeploy) {
          transitionTo(DeployState.DEPLOYING);
          deployIO.setVoltage(deployVoltage.get());
        } else if (!shootingPressureMode
            && stallTimer.hasElapsed(IntakeConstants.deployInrushIgnoreTime)
            && deployInputs.statorCurrentAmps > retractStallCurrentThreshold.get()) {
          transitionTo(DeployState.RETRACTED);
          deployIO.stop();
        } else {
          deployIO.setVoltage(retractVoltage.get());
        }
        break;

      case REHOME_DEPLOYING:
        if (goalWantsDeploy) {
          transitionTo(DeployState.DEPLOYING);
          deployIO.setVoltage(deployVoltage.get());
        } else if (stallTimer.hasElapsed(rehomeDeployTime.get())) {
          transitionTo(DeployState.RETRACTING);
          deployIO.setVoltage(retractVoltage.get());
        } else {
          deployIO.setVoltage(deployVoltage.get());
        }
        break;
    }

    // Update roller gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> rollerIO.setGains(values[0], values[1], values[2]),
        rollerKp,
        rollerKv,
        rollerKs);

    // Roller control: run when deployed; also keep running during first 90% of retraction
    boolean retractingWithRollers =
        deployState == DeployState.RETRACTING
            && deployInputs.positionRad > deployedPositionRad * retractRollerShutoffFraction.get();

    if (deployState == DeployState.DEPLOYED) {
      // Determine commanded velocity for current goal
      double commandedVelRadPerSec = 0.0;
      boolean rollerActive = false;
      switch (goal) {
        case INTAKE:
          commandedVelRadPerSec = rpmToRadPerSec(rollerIntakeRPM.get());
          rollerActive = true;
          break;
        case CLUMP_INTAKE:
          commandedVelRadPerSec = rpmToRadPerSec(rollerClumpIntakeRPM.get());
          rollerActive = true;
          break;
        case EJECT:
          commandedVelRadPerSec = rpmToRadPerSec(rollerEjectRPM.get());
          rollerActive = true;
          break;
        default:
          break;
      }

      // Jam detection: high current + near-zero velocity while motor is commanded
      boolean stallCondition =
          rollerActive
              && rollerInputs.statorCurrentAmps > rollerJamCurrentThreshold.get()
              && Math.abs(rollerInputs.velocityRadPerSec) < Math.abs(commandedVelRadPerSec) * 0.75;

      if (!stallCondition) {
        rollerJamTimer.restart();
      }

      // Handle jam state
      if (rollerJammed) {
        // Stopped — waiting for driver to release intake goal
        rollerIO.stop();
      } else if (rollerActive && rollerJamTimer.hasElapsed(rollerJamDebounceTime.get())) {
        // Jam detected — stop roller
        rollerJammed = true;
        rollerIO.stop();
      } else if (rollerActive) {
        rollerIO.setVelocity(commandedVelRadPerSec);
      } else {
        rollerIO.stop();
        rollerJammed = false;
      }
    } else if (retractingWithRollers) {
      rollerIO.setVelocity(rpmToRadPerSec(rollerIntakeRPM.get()));
    } else {
      rollerIO.stop();
      rollerJammed = false;
    }

    updateAlerts();
  }

  private void transitionTo(DeployState newState) {
    // Switch to coast when deployed so intake can be pushed back
    if (newState == DeployState.DEPLOYED) {
      deployIO.setBrakeMode(false);
      deployedPositionRad = deployInputs.positionRad;
      positionCorrectionActive = false;
      positionCorrectionGaveUp = false;
    } else if (deployState == DeployState.DEPLOYED) {
      deployIO.setBrakeMode(true);
    }
    if (newState == DeployState.RETRACTED) {
      retractedPositionRad = deployInputs.positionRad;
    }
    deployState = newState;
    stallTimer.restart();
  }

  @AutoLogOutput(key = "Intake/Roller/StatorCurrentAmps")
  public double getRollerStatorCurrentAmps() {
    return rollerInputs.statorCurrentAmps;
  }

  @AutoLogOutput(key = "Intake/Roller/Jammed")
  public boolean isRollerJammed() {
    return rollerJammed;
  }

  private void updateAlerts() {
    deployDisconnectedAlert.set(!deployInputs.connected && Constants.currentMode != Mode.SIM);
    rollerDisconnectedAlert.set(!rollerInputs.connected && Constants.currentMode != Mode.SIM);
    deployOverTempAlert.set(deployInputs.tempCelsius > 80.0);
    rollerOverTempAlert.set(rollerInputs.tempCelsius > 80.0);
    rollerJamAlert.set(rollerJammed);
  }

  private static double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  // Command factories
  public Command setGoalCommand(Goal goal) {
    return V3Commands.named(V3Commands.runOnce(() -> setGoal(goal)), "Intake " + goal.name());
  }

  public Command intakeCommand() {
    return V3Commands.named(V3Commands.startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE), this)
        , "Intake");
  }

  /**
   * Periodically triggers a rehome cycle every 1.5 seconds. Intended for use during auto shooting
   * phases to keep the intake seated. Does not require the intake subsystem so it can run in
   * parallel with other intake goal commands.
   */
  public Command periodicAutoRehomeCommand() {
    return Command.noRequirements(co -> {
      while (true) {
        co.wait(org.wpilib.units.Units.Seconds.of(1.5));
        setGoal(Goal.IDLE);
        co.yield(); // Preserve the original repeat boundary's scheduler cycle.
      }
    }).named("Intake Periodic Auto Rehome");
  }

  public Command ejectCommand() {
    return V3Commands.named(V3Commands.startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE), this)
        , "Eject");
  }

  /**
   * Ramps voltage on the roller motor and collects velocity/voltage samples, then calculates kS and
   * kV via linear regression. Run from the auto chooser, cancel to see results.
   */
  public static Command rollerFFCharacterization(Intake intake) {
    double rampRate = 0.5; // Volts per second
    double startDelay = 1.0; // Seconds to let deploy settle
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return V3Commands.sequence(
        // Deploy the intake first and wait for it to settle
        V3Commands.runOnce(() -> intake.setGoal(Goal.DEPLOYED_IDLE)),
        V3Commands.waitSeconds(startDelay),

        // Reset data
        V3Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Start timer
        V3Commands.runOnce(timer::restart),

        // Ramp voltage and collect samples
        V3Commands.finallyDo(V3Commands.run(
                () -> {
                  double voltage = timer.get() * rampRate;
                  intake.runRollerCharacterization(voltage);
                  velocitySamples.add(intake.getRollerCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                intake),
                () -> {
                  intake.runRollerCharacterization(0.0);
                  intake.setGoal(Goal.IDLE);

                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println(
                      "********** Intake Roller FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
