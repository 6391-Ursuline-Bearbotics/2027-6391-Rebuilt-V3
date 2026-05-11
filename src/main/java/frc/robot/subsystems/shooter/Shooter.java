package frc.robot.subsystems.shooter;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.AlertType;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Trench;
import frc.robot.GameData;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends Mechanism {
  // Tunable PID gains
  private static final LoggedTunableNumber shooterKp = new LoggedTunableNumber("Shooter/kP", 0.1);
  private static final LoggedTunableNumber shooterKv = new LoggedTunableNumber("Shooter/kV", 0.12);
  private static final LoggedTunableNumber shooterKs = new LoggedTunableNumber("Shooter/kS", 0.0);

  // Tunable velocity setpoints
  private static final LoggedTunableNumber ejectRPM =
      new LoggedTunableNumber("Shooter/EjectRPM", ShooterConstants.ejectRPM);
  private static final LoggedTunableNumber toleranceRPM =
      new LoggedTunableNumber("Shooter/ToleranceRPM", ShooterConstants.toleranceRPM);
  private static final LoggedTunableNumber passToleranceRPM =
      new LoggedTunableNumber("Shooter/PassToleranceRPM", ShooterConstants.passToleranceRPM);

  // RPM override for manual testing (0 = use distance table)
  private static final LoggedTunableNumber rpmOverride =
      new LoggedTunableNumber("Shooter/RPMOverride", 0.0);

  // Hood angle override for servo testing (0 = use distance table, non-zero = use this angle)
  private static final LoggedTunableNumber hoodAngleOverride =
      new LoggedTunableNumber("Shooter/HoodAngleOverride", 0.0);

  // Hybrid control: bang-bang spin-up, PID hold, bang-bang while feeding
  // (1.0 = hybrid enabled, 0.0 = pure PID always)
  private static final LoggedTunableNumber hybridControlEnabled =
      new LoggedTunableNumber("Shooter/HybridControlEnabled", 1.0);
  private static final LoggedTunableNumber bangBangPeakAmps =
      new LoggedTunableNumber("Shooter/BangBangPeakAmps", ShooterConstants.bangBangPeakCurrentAmps);

  // Enable/disable shoot-on-the-move compensation
  private static final LoggedTunableNumber shootOnMoveEnabled =
      new LoggedTunableNumber("Shooter/ShootOnMoveEnabled", 1.0);

  // Pitch compensation
  private static final double kPitchDeadZoneRad = 0.045;
  private static final LoggedTunableNumber pitchMultiplier =
      new LoggedTunableNumber("Shooter/PitchMultiplier", 1.0);

  // Tunable jam detection parameters
  private static final LoggedTunableNumber jamCurrentThreshold =
      new LoggedTunableNumber("Shooter/JamCurrentThreshold", ShooterConstants.jamCurrentThreshold);
  private static final LoggedTunableNumber jamDebounceTime =
      new LoggedTunableNumber("Shooter/JamDebounceTime", ShooterConstants.jamDebounceTime);

  // Tunable trench approach parameters
  private static final LoggedTunableNumber trenchApproachDistance =
      new LoggedTunableNumber(
          "Shooter/TrenchApproachDistance", ShooterConstants.trenchApproachDistanceMeters);
  private static final LoggedTunableNumber trenchApproachMinVelocity =
      new LoggedTunableNumber(
          "Shooter/TrenchApproachMinVelocity", ShooterConstants.trenchApproachMinVelocityMps);
  private static final LoggedTunableNumber trenchApproachXMargin =
      new LoggedTunableNumber(
          "Shooter/TrenchApproachXMargin", ShooterConstants.trenchApproachXMarginMeters);

  public enum Goal {
    IDLE,
    SHOOT,
    PASS,
    EJECT
  }

  // IO
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterHoodIO hoodIO;
  private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

  // Pose, speed, and pitch suppliers from drive
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final Supplier<Double> pitchSupplier;

  // Returns true while any shot button is physically held on either controller
  private final BooleanSupplier shotButtonHeldSupplier;

  // Interpolation tables — hub shots
  private final InterpolatingDoubleTreeMap hubDistanceToRPM;
  private final InterpolatingDoubleTreeMap hubDistanceToAngle;
  // Interpolation tables — passing shots
  private final InterpolatingDoubleTreeMap passDistanceToRPM;
  private final InterpolatingDoubleTreeMap passDistanceToAngle;
  // Shared time-of-flight table
  private final InterpolatingDoubleTreeMap distanceToTOF;

  // State
  private Goal goal = Goal.IDLE;
  private double commandedRPM = 0.0;
  private double commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
  private double hoodAngleCommandDeg = 0.0; // 0 = not active
  private double distanceToTarget = 0.0;
  private Translation2d aimTarget = Translation2d.kZero;

  // Hybrid control state — latches true once at setpoint, resets on goal change
  private boolean spunUp = false;
  private final Supplier<Boolean> indexerFeedingSupplier;

  // Jam detection
  private final Timer jamTimer = new Timer();
  private boolean jammed = false;

  // Manual distance setpoint (0 = use vision/pose-calculated distance)
  private double distanceSetpointMeters = 0;

  // Alerts
  private final Alert leftDisconnectedAlert =
      new Alert("Shooter left motor disconnected.", AlertType.kError);
  private final Alert rightDisconnectedAlert =
      new Alert("Shooter right motor disconnected.", AlertType.kError);
  private final Alert leftOverTempAlert =
      new Alert("Shooter left motor over temperature.", AlertType.kWarning);
  private final Alert rightOverTempAlert =
      new Alert("Shooter right motor over temperature.", AlertType.kWarning);
  private final Alert jamAlert = new Alert("Shooter jam detected!", AlertType.kError);

  public Shooter(
      ShooterIO io,
      ShooterHoodIO hoodIO,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier,
      Supplier<Boolean> indexerFeedingSupplier,
      Supplier<Double> pitchSupplier,
      BooleanSupplier shotButtonHeldSupplier) {
    this.io = io;
    this.hoodIO = hoodIO;
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    this.indexerFeedingSupplier = indexerFeedingSupplier;
    this.pitchSupplier = pitchSupplier;
    this.shotButtonHeldSupplier = shotButtonHeldSupplier;
    this.hubDistanceToRPM = ShooterConstants.createHubDistanceToRPMMap();
    this.hubDistanceToAngle = ShooterConstants.createHubDistanceToAngleMap();
    this.passDistanceToRPM = ShooterConstants.createPassDistanceToRPMMap();
    this.passDistanceToAngle = ShooterConstants.createPassDistanceToAngleMap();
    this.distanceToTOF = ShooterConstants.createDistanceToTOFMap();
    jamTimer.start();
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
    if (goal == Goal.SHOOT || goal == Goal.PASS) {
      hoodAngleCommandDeg = 0.0;
    }
  }

  /** Directly commands the hood to a specific angle, overriding goal-based control. */
  public void setHoodAngle(double degrees) {
    hoodAngleCommandDeg = degrees;
  }

  /** Clears the direct hood angle command, returning control to goal-based logic. */
  public void clearHoodAngle() {
    hoodAngleCommandDeg = 0.0;
  }

  @AutoLogOutput(key = "Shooter/Goal")
  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput(key = "Shooter/CommandedRPM")
  public double getCommandedRPM() {
    return commandedRPM;
  }

  @AutoLogOutput(key = "Shooter/CommandedAngleDeg")
  public double getCommandedAngleDeg() {
    return commandedAngleDeg;
  }

  @AutoLogOutput(key = "Shooter/DistanceToTarget")
  public double getDistanceToTarget() {
    return distanceToTarget;
  }

  /** Returns the current aim target (virtual target after shoot-on-the-move compensation). */
  public Translation2d getAimTarget() {
    return aimTarget;
  }

  /** Sets the manual distance setpoint in meters. */
  public void setDistanceSetpoint(double meters) {
    distanceSetpointMeters = Math.max(0.5, meters);
  }

  /** Adjusts the manual distance setpoint by a delta in meters. */
  public void adjustDistanceSetpoint(double deltaMeters) {
    setDistanceSetpoint(distanceSetpointMeters + deltaMeters);
  }

  @AutoLogOutput(key = "Shooter/DistanceSetpointFt")
  public double getDistanceSetpointFt() {
    return Units.metersToFeet(distanceSetpointMeters);
  }

  @AutoLogOutput(key = "Shooter/AverageVelocityRPM")
  public double getAverageVelocityRPM() {
    double avgRadPerSec = (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
    return radPerSecToRPM(avgRadPerSec);
  }

  /** Returns true if both motors are within tolerance of the commanded setpoint. */
  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean isAtSetpoint() {
    if ((goal != Goal.SHOOT && goal != Goal.PASS) || commandedRPM == 0.0) {
      return false;
    }
    double tol = goal == Goal.PASS ? passToleranceRPM.get() : toleranceRPM.get();
    double tolerance = rpmToRadPerSec(tol);
    double setpoint = rpmToRadPerSec(commandedRPM);
    return Math.abs(inputs.leftVelocityRadPerSec - setpoint) < tolerance
        && Math.abs(inputs.rightVelocityRadPerSec - setpoint) < tolerance;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);

    if (DriverStation.isDisabled()) {
      io.stop();
      hoodIO.stop();
      updateAlerts();
      return;
    }

    // Always update distance/aim target so logging reflects current value regardless of goal
    updateAimTarget();

    // Update gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> io.setGains(values[0], values[1], values[2]),
        shooterKp,
        shooterKv,
        shooterKs);
    LoggedTunableNumber.ifChanged(
        hashCode() + 1, values -> io.setPeakTorqueCurrent(values[0]), bangBangPeakAmps);

    // Clear jam flag when goal changes to an active state (operator re-engages)
    if ((goal == Goal.SHOOT || goal == Goal.PASS || goal == Goal.EJECT) && jammed) {
      jammed = false;
      jamTimer.restart();
    }

    switch (goal) {
      case SHOOT:
        commandedRPM = calculateShootRPM(hubDistanceToRPM);
        commandedAngleDeg = hubDistanceToAngle.get(distanceToTarget);
        if (GameData.canSpinUp(poseSupplier.get().getTranslation())) {
          // Latch spunUp once at setpoint (resets when goal changes away from SHOOT)
          if (isAtSetpoint()) {
            spunUp = true;
          }

          boolean useHybrid = hybridControlEnabled.get() > 0.5;
          boolean useBangBang;
          if (!useHybrid) {
            // Pure PID mode
            useBangBang = false;
          } else if (!spunUp) {
            // Spin-up phase: bang-bang for max torque
            useBangBang = true;
          } else {
            // At speed: bang-bang while feeding, PID while holding
            useBangBang = indexerFeedingSupplier.get();
          }

          if (useBangBang) {
            io.setVelocityFOC(rpmToRadPerSec(commandedRPM));
          } else {
            io.setVelocity(rpmToRadPerSec(commandedRPM));
          }
          Logger.recordOutput("Shooter/ControlMode", useBangBang ? "BangBang" : "PID");
        } else {
          commandedRPM = 0.0;
          commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
          spunUp = false;
          io.stop();
        }
        break;
      case PASS:
        commandedRPM = calculateShootRPM(passDistanceToRPM);
        commandedAngleDeg = passDistanceToAngle.get(distanceToTarget);
        if (isAtSetpoint()) {
          spunUp = true;
        }
        io.setVelocity(rpmToRadPerSec(commandedRPM));
        Logger.recordOutput("Shooter/ControlMode", "PID");
        break;
      case EJECT:
        commandedRPM = ejectRPM.get();
        commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
        spunUp = false;
        io.setVelocity(rpmToRadPerSec(commandedRPM));
        break;
      default:
        commandedRPM = 0.0;
        commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
        spunUp = false;
        io.stop();
        break;
    }

    // Jam detection: high current + near-zero velocity on either motor while actively commanded
    if (goal == Goal.SHOOT || goal == Goal.PASS || goal == Goal.EJECT) {
      double commandedVel = rpmToRadPerSec(commandedRPM);
      boolean leftStall =
          inputs.leftStatorCurrentAmps > jamCurrentThreshold.get()
              && Math.abs(inputs.leftVelocityRadPerSec) < Math.abs(commandedVel) * 0.1;
      boolean rightStall =
          inputs.rightStatorCurrentAmps > jamCurrentThreshold.get()
              && Math.abs(inputs.rightVelocityRadPerSec) < Math.abs(commandedVel) * 0.1;

      if (!leftStall && !rightStall) {
        jamTimer.restart();
      }

      if (commandedRPM != 0.0 && jamTimer.hasElapsed(jamDebounceTime.get())) {
        // Jam detected — stop motors, go to IDLE
        jammed = true;
        goal = Goal.IDLE;
        commandedRPM = 0.0;
        commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
        io.stop();
      }
    } else {
      jamTimer.restart();
    }

    // Apply hood angle override for servo testing (non-zero value bypasses distance table)
    if (hoodAngleOverride.get() != 0.0) {
      commandedAngleDeg = hoodAngleOverride.get();
    }

    // Apply direct hood angle command (e.g. from auto routines before trench entry)
    if (hoodAngleCommandDeg != 0.0) {
      commandedAngleDeg = hoodAngleCommandDeg;
    }

    // Trench approach override — lowers hood to 26° when driving into a trench during teleop.
    // Skipped in autonomous (auto routines manage hood angle explicitly) and while any shot
    // button is physically held on either controller.
    if (DriverStation.isTeleop()
        && !shotButtonHeldSupplier.getAsBoolean()
        && isApproachingTrench()) {
      commandedAngleDeg = 26.0;
    }

    hoodIO.setAngle(commandedAngleDeg);

    // Log the aim target and hub status for visualization
    Logger.recordOutput(
        "Shooter/AimTarget", new Pose2d(aimTarget, poseSupplier.get().getRotation()));
    Logger.recordOutput("Shooter/HubActive", GameData.isHubActive());

    updateAlerts();
  }

  /**
   * Updates distanceToTarget and aimTarget every loop so the logged value always reflects the
   * current robot pose, regardless of whether the shooter goal is active.
   */
  private void updateAimTarget() {
    if (distanceSetpointMeters > 0.0) {
      distanceToTarget = distanceSetpointMeters;
      return;
    }

    Pose2d robotPose = poseSupplier.get();
    Translation2d robotPosition = robotPose.getTranslation();
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Determine real target (hub or passing target)
    Translation2d realTarget;
    if (FieldConstants.isInOwnAllianceZone(robotPosition, isRedAlliance)) {
      realTarget = FieldConstants.getHubCenter(isRedAlliance);
    } else {
      realTarget = FieldConstants.getPassingTarget(robotPosition, isRedAlliance);
    }

    // Apply shoot-on-the-move compensation using virtual target method
    Translation2d compensatedTarget = realTarget;
    if (shootOnMoveEnabled.get() > 0.5) {
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
      compensatedTarget =
          computeVirtualTarget(
              robotPosition, realTarget, fieldSpeeds, ShooterConstants.shotCompensationIterations);
    }

    aimTarget = compensatedTarget;
    distanceToTarget = robotPosition.getDistance(compensatedTarget);
  }

  /** Returns the target RPM based on the current distanceToTarget (updated each loop). */
  private double calculateShootRPM(InterpolatingDoubleTreeMap rpmTable) {
    if (rpmOverride.get() != 0.0) {
      return rpmOverride.get();
    }

    // Radial velocity compensation: when the robot moves away from the hub the ball has less
    // net speed toward it, so look up RPM for a larger effective distance. Gated by the same
    // shootOnMoveEnabled tunable as the lateral (heading) compensation.
    double effectiveDistance = distanceToTarget;
    if (shootOnMoveEnabled.get() > 0.5) {
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
      Pose2d robotPose = poseSupplier.get();
      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      Translation2d toHub = FieldConstants.getHubCenter(isRed).minus(robotPose.getTranslation());
      double dist = toHub.getNorm();
      if (dist > 0.01) {
        // Unit vector from robot toward hub
        double ux = toHub.getX() / dist;
        double uy = toHub.getY() / dist;
        // Positive = moving toward hub, negative = moving away
        double vRadialTowardHub =
            fieldSpeeds.vxMetersPerSecond * ux + fieldSpeeds.vyMetersPerSecond * uy;
        double tof = distanceToTOF.get(distanceToTarget);
        // Moving away reduces ball speed toward hub — compensate by boosting effective distance
        effectiveDistance = distanceToTarget - vRadialTowardHub * tof;
      }
    }
    Logger.recordOutput("Shooter/EffectiveDistance", effectiveDistance);

    double baseRPM = rpmTable.get(effectiveDistance);

    // Apply pitch compensation outside the dead zone
    double pitch = pitchSupplier.get();
    if (Math.abs(pitch) > kPitchDeadZoneRad) {
      double adjustment = baseRPM * (Math.abs(pitch) * pitchMultiplier.get());
      baseRPM += pitch < 0 ? -adjustment : adjustment;
    }
    Logger.recordOutput("Shooter/PitchCompensatedRPM", baseRPM);

    return baseRPM;
  }

  /**
   * Computes the virtual target position to compensate for robot motion. Uses iterative refinement:
   * each iteration recomputes the time-of-flight at the new distance and adjusts the target.
   */
  private Translation2d computeVirtualTarget(
      Translation2d robotPosition,
      Translation2d realTarget,
      ChassisSpeeds fieldSpeeds,
      int iterations) {
    Translation2d virtualTarget = realTarget;
    for (int i = 0; i < iterations; i++) {
      double distance = robotPosition.getDistance(virtualTarget);
      double tof = distanceToTOF.get(distance);
      virtualTarget =
          new Translation2d(
              realTarget.getX() - fieldSpeeds.vxMetersPerSecond * tof,
              realTarget.getY() - fieldSpeeds.vyMetersPerSecond * tof);
    }
    return virtualTarget;
  }

  /**
   * Returns true when the robot is moving toward a trench and close enough that the hood must be
   * lowered proactively to ensure clearance.
   */
  @AutoLogOutput(key = "Shooter/TrenchApproachActive")
  private boolean isApproachingTrench() {
    Translation2d robotPos = poseSupplier.get().getTranslation();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    double approachDist = trenchApproachDistance.get();
    double minVel = trenchApproachMinVelocity.get();
    double xMargin = trenchApproachXMargin.get();
    double halfOpening = Trench.openingWidth / 2.0 + xMargin;

    // Check all 4 trenches
    Translation2d[] trenchCenters = {
      Trench.blueTopCenter, Trench.blueBottomCenter, Trench.redTopCenter, Trench.redBottomCenter
    };
    double[] entryYs = {
      Trench.topTrenchEntryY,
      Trench.bottomTrenchEntryY,
      Trench.topTrenchEntryY,
      Trench.bottomTrenchEntryY
    };
    // Direction toward wall: +Y for top trenches, -Y for bottom trenches
    double[] wallDirections = {1.0, -1.0, 1.0, -1.0};

    for (int i = 0; i < trenchCenters.length; i++) {
      // Check X bounds: robot must be within opening width + margin of trench center
      if (Math.abs(robotPos.getX() - trenchCenters[i].getX()) > halfOpening) {
        continue;
      }

      // Check Y distance to trench entry
      double distToEntry = Math.abs(robotPos.getY() - entryYs[i]);
      if (distToEntry > approachDist) {
        continue;
      }

      // Check velocity toward the trench wall (dot product with wall direction)
      double velocityTowardTrench = fieldSpeeds.vyMetersPerSecond * wallDirections[i];
      if (velocityTowardTrench > minVel) {
        return true;
      }

      // Also trigger if already inside the trench (past the entry Y toward the wall)
      boolean insideTrench =
          (wallDirections[i] > 0 && robotPos.getY() > entryYs[i])
              || (wallDirections[i] < 0 && robotPos.getY() < entryYs[i]);
      if (insideTrench) {
        return true;
      }
    }
    return false;
  }

  @AutoLogOutput(key = "Shooter/Jammed")
  public boolean isJammed() {
    return jammed;
  }

  /** Returns the current hood angle in degrees as reported by the servo feedback. */
  public double getHoodAngleDeg() {
    return hoodInputs.positionDeg;
  }

  /** Returns true if the hood is within toleranceDeg of the target angle. */
  public boolean isHoodAtAngle(double targetDeg, double toleranceDeg) {
    return Math.abs(hoodInputs.positionDeg - targetDeg) < toleranceDeg;
  }

  /** Returns true if the hood is at or below 26 degrees. */
  @AutoLogOutput(key = "Shooter/HoodAtOrBelow26Deg")
  public boolean isHoodAtOrBelow26Deg() {
    return hoodInputs.positionDeg <= 26.0;
  }

  private void updateAlerts() {
    leftDisconnectedAlert.set(!inputs.leftConnected && Constants.currentMode != Mode.SIM);
    rightDisconnectedAlert.set(!inputs.rightConnected && Constants.currentMode != Mode.SIM);
    leftOverTempAlert.set(inputs.leftTempCelsius > 80.0);
    rightOverTempAlert.set(inputs.rightTempCelsius > 80.0);
    jamAlert.set(jammed);
  }

  private static double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  private static double radPerSecToRPM(double radPerSec) {
    return radPerSec * 60.0 / (2.0 * Math.PI);
  }

  // Command factories
  public Command setGoalCommand(Goal goal) {
    return Commands.runOnce(() -> setGoal(goal)).withName("Shooter " + goal.name());
  }

  public Command shootCommand() {
    return Commands.startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE), this)
        .withName("Shooter Shoot");
  }

  public Command passCommand() {
    return Commands.startEnd(() -> setGoal(Goal.PASS), () -> setGoal(Goal.IDLE), this)
        .withName("Shooter Pass");
  }

  public Command ejectCommand() {
    return Commands.startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE), this)
        .withName("Shooter Eject");
  }

  /** Run shooter motors at a raw voltage for characterization. Bypasses goal logic. */
  public void runCharacterization(double volts) {
    io.setVoltage(volts);
  }

  /** Returns the average velocity of both shooter motors in rad/s for characterization. */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  }

  /**
   * Ramps voltage on the shooter motors and collects velocity/voltage samples, then calculates kS
   * and kV via linear regression. Run from the auto chooser, cancel to see results.
   */
  public static Command shooterFFCharacterization(Shooter shooter) {
    double rampRate = 0.25; // Volts per second (slower for high-inertia flywheel)
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Start timer
        Commands.runOnce(timer::restart),

        // Ramp voltage and collect samples
        Commands.run(
                () -> {
                  double voltage = timer.get() * rampRate;
                  shooter.runCharacterization(voltage);
                  velocitySamples.add(shooter.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                shooter)
            .finallyDo(
                () -> {
                  shooter.runCharacterization(0.0);

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
                  System.out.println("********** Shooter FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
