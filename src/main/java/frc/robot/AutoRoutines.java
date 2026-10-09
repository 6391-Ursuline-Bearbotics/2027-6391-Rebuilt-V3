package frc.robot;

import static org.wpilib.units.Units.Seconds;

import frc.robot.auto.AutoFactory;
import frc.robot.auto.AutoRoutine;
import frc.robot.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Consumer;
import org.wpilib.command3.Coroutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
  // How long to feed the indexer when shooting in auto (smaller hopper = shorter shoot)
  private static final LoggedTunableNumber shootDurationSecs =
      new LoggedTunableNumber("Auto/ShootDurationSecs", 4.0);

  // Max speed (m/s) while PIDing to the staging pose during the shoot sequence.
  private static final LoggedTunableNumber trenchStagingSpeedMps =
      new LoggedTunableNumber("Auto/TrenchStagingSpeedMps", 0.5);

  // Staging poses the robot PID drives toward while shooting, positioning it at the trench
  // entrance before the next collection pass. Outpost = bottom wall (low Y), Depot = top wall.
  private static final LoggedTunableNumber trenchOutpostStagingX =
      new LoggedTunableNumber("Auto/TrenchOutpostStagingX", 3.0);
  private static final LoggedTunableNumber trenchOutpostStagingY =
      new LoggedTunableNumber("Auto/TrenchOutpostStagingY", 0.75);
  private static final LoggedTunableNumber trenchDepotStagingX =
      new LoggedTunableNumber("Auto/TrenchDepotStagingX", 3.0);
  private static final LoggedTunableNumber trenchDepotStagingY =
      new LoggedTunableNumber("Auto/TrenchDepotStagingY", 7.55);

  // Extra delay (seconds) after shoot-first preload shot before starting the trajectory.
  // Configured as a number on the dashboard so it works at competition without tuning mode.
  // How long to settle after shooter reaches setpoint before feeding (lets heading PID converge).
  private static final LoggedTunableNumber shootSettleSecs =
      new LoggedTunableNumber("Auto/ShootSettleSecs", 0.5);

  private static final org.wpilib.tunable.TunableDouble shootFirstDelaySecs =
      org.wpilib.tunable.TunableDouble.createConfig(0.0,
          org.wpilib.tunable.TunableConfig.of(org.wpilib.tunable.TunableOption.ROBUST)
              .withProperty("persistent", "true"));

  static {
    org.wpilib.tunable.Tunables.getTable("Autonomous")
        .publish("ShootFirstDelaySecs", shootFirstDelaySecs);
    frc.robot.util.HubConfiguration.register(
        "Autonomous/ShootFirstDelaySecs", shootFirstDelaySecs::get);
  }

  // Gather clump detection: roller stator amps above this threshold triggers slow-down.
  private static final LoggedTunableNumber gatherClumpCurrentAmps =
      new LoggedTunableNumber("Auto/GatherClumpCurrentAmps", 25.0);
  // Speed cap (m/s) applied when a clump is detected during a gather pass.
  private static final LoggedTunableNumber gatherSlowSpeedMps =
      new LoggedTunableNumber("Auto/GatherSlowSpeedMps", 1.2);

  // Speed (m/s) to creep toward the alliance wall when no AprilTags are visible after crossing back
  // over the bump. Odometry drift from the bump can leave the robot closer to the hub than it
  // thinks; creeping toward the wall brings it into camera range so vision can correct the pose.
  private static final LoggedTunableNumber bumpCreepSpeedMps =
      new LoggedTunableNumber("Auto/BumpCreepSpeedMps", 0.3);

  // Seconds elapsed from routine activation at which the shoot phase ends and the bump rush begins.
  // The minimum shootDurationSecs is always completed first; this only extends beyond that.
  private static final LoggedTunableNumber bumpRushAutoTimeSecs =
      new LoggedTunableNumber("Auto/BumpRushAutoTimeSecs", 17.5);

  private final AutoFactory factory;
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Vision vision;

  public AutoRoutines(
      AutoFactory factory,
      Drive drive,
      Intake intake,
      Indexer indexer,
      Shooter shooter,
      Vision vision) {
    this.factory = factory;
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.vision = vision;
  }

  public AutoRoutine resetOdometryStart() {
    AutoRoutine routine = newRoutine("ResetOdometryStart");
    routine.run(co -> {
      co.await(factory.resetOdometry(() -> Optional.of(new Pose2d(3.559, 4.0296, Rotation2d.ZERO))));
      System.out.println("Odometry reset to start position");
    });
    return routine;
  }

  public AutoRoutine depotCycle() {
    AutoRoutine routine = newRoutine("Depot Cycle");
    AutoTrajectory path = routine.trajectory("DepotCycle");
    routine.run(co -> {
      co.await(path.resetOdometry());
      co.await(path.cmd());
    });
    return routine;
  }

  public AutoRoutine depotInside() {
    AutoRoutine routine = newRoutine("Depot Inside");
    AutoTrajectory path = routine.trajectory("DepotInside");
    routine.run(co -> {
      co.await(path.resetOdometry());
      co.await(path.cmd());
    });
    return routine;
  }

  public AutoRoutine outpostDoublePass() {
    return buildDoublePass(
        "Outpost Double Pass", "OutpostBump", "OutpostDoublePass", "OutpostBumpReturn", false);
  }

  public AutoRoutine outpostDoublePassShootFirst() {
    return buildDoublePass(
        "Outpost Double Pass Shoot First",
        "OutpostBump",
        "OutpostDoublePass",
        "OutpostBumpReturn",
        true);
  }

  public AutoRoutine depotDoublePass() {
    return buildDoublePass(
        "Depot Double Pass", "DepotBump", "DepotDoublePass", "DepotBumpReturn", false);
  }

  public AutoRoutine depotDoublePassShootFirst() {
    return buildDoublePass(
        "Depot Double Pass Shoot First", "DepotBump", "DepotDoublePass", "DepotBumpReturn", true);
  }

  public AutoRoutine outpostFullPass() {
    return buildDoublePass(
        "Outpost FULL Pass", "OutpostBump", "FullOutpostSinglePass", "OutpostBumpReturn", false);
  }

  public AutoRoutine outpostSinglePass() {
    return buildDoublePass(
        "Outpost Single Pass", "OutpostBump", "OutpostSinglePass", "OutpostBumpReturn", false);
  }

  public AutoRoutine outpostSinglePassShootFirst() {
    return buildDoublePass(
        "Outpost Single Pass Shoot First",
        "OutpostBump",
        "OutpostSinglePass",
        "OutpostBumpReturn",
        true);
  }

  public AutoRoutine depotSinglePass() {
    return buildDoublePass(
        "Depot Single Pass", "DepotBump", "DepotSinglePass", "DepotBumpReturn", false);
  }

  public AutoRoutine depotSinglePassShootFirst() {
    return buildDoublePass(
        "Depot Single Pass Shoot First", "DepotBump", "DepotSinglePass", "DepotBumpReturn", true);
  }

  /**
   * Depot Single Pass - Shoot On Move variant. Crosses bump out, collects balls on the single pass
   * trajectory, returns over the bump, then shoots while rolling toward the depot at 0.5 m/s from
   * the alliance side. Deploys intake when ~2m from the depot to collect staged balls while still
   * firing. Runs until auto ends.
   */
  public AutoRoutine depotSinglePassShootOnMove() {
    AutoRoutine routine = newRoutine("Depot Single Pass Shoot On Move");
    AutoTrajectory bump = routine.trajectory("DepotBump");
    AutoTrajectory singlePass = routine.trajectory("DepotSinglePass");
    AutoTrajectory bumpReturn = routine.trajectory("DepotBumpReturn");
    routine.run(co -> {
      co.await(bump.resetOdometry());
      intake.setGoal(Intake.Goal.INTAKE);
      co.await(bump.cmd());
      co.await(sprintToPose(singlePass.getInitialPose().orElse(new Pose2d())).withTimeout(Seconds.of(3)));
      co.await(singlePass.cmd());
      intake.setGoal(Intake.Goal.IDLE);
      shooter.setGoal(Shooter.Goal.SHOOT);
      co.await(bumpReturn.cmd());
      co.await(sprintToPose(bumpReturn.getFinalPose().orElse(new Pose2d())).withTimeout(Seconds.of(2)));

      // Aiming, readiness/feeding, and intake work share the routine's lifetime.
      co.fork(driveTowardDepotAimingAtHub(0.5));
      co.fork(Command.noRequirements(feed -> {
        feed.waitUntil(() -> shooter.isAtSetpoint() || !canScore());
        if (canScore()) {
          feed.fork(indexer.feedCommand());
          feed.waitUntil(() -> !canScore());
          indexer.setGoal(Indexer.Goal.IDLE);
        }
      }).named("Wait For Moving Shot"));
      Command rehome = intake.periodicAutoRehomeCommand();
      co.fork(rehome);
      co.waitUntil(() -> drive.getPose().getTranslation()
          .getDistance(FieldConstants.getDepotCenter(isRed())) < 2.0);
      co.scheduler().cancel(rehome); // Stop jostling before deploying, never concurrently.
      intake.setGoal(Intake.Goal.INTAKE);
      co.park();
    });
    return routine;
  }

  public AutoRoutine safe() {
    return buildSafe("Safe", false);
  }

  public AutoRoutine safeShootFirst() {
    return buildSafe("Safe Shoot First", true);
  }

  public AutoRoutine shootOnly() {
    AutoRoutine routine = newRoutine("Shoot Only");
    routine.run(co -> {
      co.await(scoringPhase("Shoot Only", shot -> {
        shooter.setGoal(Shooter.Goal.SHOOT);
        shot.await(aimBackAtHub().withTimeout(Seconds.of(1.5)));
        indexer.setGoal(Indexer.Goal.FEED);
        shot.await(intake.periodicAutoRehomeCommand().withTimeout(Seconds.of(10)));
      }));
      stopShooting();
    });
    return routine;
  }

  /**
   * Trench Outpost Disrupt: Runs TrenchOutpostDisrupt trajectory (rams opponent), deploys intake at
   * "Intake" waypoint, spins up shooter at "Spin" waypoint. After trajectory ends, aims and shoots
   * (stationary or creeping toward Points start based on Auto/TrenchShootOnMove tunable). Then runs
   * TrenchOutpostPoints for a second scoring pass.
   */
  public AutoRoutine trenchOutpostDisrupt() {
    return buildTrenchDisrupt(
        "Trench Outpost Disrupt",
        routine -> routine.trajectory("TrenchOutpostDisrupt"),
        routine -> routine.trajectory("OutpostBump"));
  }

  /** Trench Depot Disrupt: Mirrors outpost disrupt to the depot side via mirrorY(). */
  public AutoRoutine trenchDepotDisrupt() {
    return buildTrenchDisrupt(
        "Trench Depot Disrupt",
        routine -> routine.trajectory("TrenchOutpostDisrupt").mirrorY(),
        routine -> routine.trajectory("DepotBump"));
  }

  /**
   * Trench Outpost Points: Runs TrenchOutpostPoints trajectory. Deploys intake at "Intake"
   * waypoint, spins up shooter at "Spin" waypoint. After trajectory ends, shoots statically (like
   * doublepass bump), then rushes back over the outpost bump.
   */
  public AutoRoutine trenchOutpostPoints() {
    return buildTrenchPoints(
        "Trench Outpost Points",
        routine -> routine.trajectory("TrenchOutpostPoints"),
        routine -> routine.trajectory("OutpostBump"));
  }

  /** Trench Depot Points: Mirrors outpost points to the depot side via mirrorY(). */
  public AutoRoutine trenchDepotPoints() {
    return buildTrenchPoints(
        "Trench Depot Points",
        routine -> routine.trajectory("TrenchOutpostPoints").mirrorY(),
        routine -> routine.trajectory("DepotBump"));
  }

  /**
   * Trench Outpost Follow: Shoots the preloaded ball first, then follows TrenchOutpostFollow
   * (intake at "Intake", shooter spinup at "Spin"), aims and shoots while staging, then gathers.
   */
  public AutoRoutine trenchOutpostFollow() {
    return buildTrenchFollow(
        "Trench Outpost Follow",
        routine -> routine.trajectory("TrenchOutpostFollow"),
        routine -> routine.trajectory("OutpostStagingGather"),
        trenchOutpostStagingX::get,
        trenchOutpostStagingY::get);
  }

  /** Trench Depot Follow: Mirrors outpost follow to the depot side via mirrorY(). */
  public AutoRoutine trenchDepotFollow() {
    return buildTrenchFollow(
        "Trench Depot Follow",
        routine -> routine.trajectory("TrenchOutpostFollow").mirrorY(),
        routine -> routine.trajectory("OutpostStagingGather").mirrorY(),
        trenchDepotStagingX::get,
        trenchDepotStagingY::get);
  }

  /**
   * Builds a trench disrupt auto. Runs the disrupt trajectory (intake at "Intake" waypoint, shooter
   * spinup at "Spin" waypoint), then does a static shot (same structure as Points), then rushes
   * back over the bump with intake deployed.
   */
  private AutoRoutine buildTrenchDisrupt(String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> disruptFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> bumpFactory) {
    AutoRoutine routine = newRoutine(name);
    AutoTrajectory path = disruptFactory.apply(routine);
    AutoTrajectory bump = bumpFactory.apply(routine);
    bindIntakeAndSpin(path);
    routine.run(co -> {
      Timer autoTimer = Timer.createStarted();
      co.await(path.resetOdometry());
      co.await(path.cmd());
      intake.setGoal(Intake.Goal.IDLE);
      co.await(staticShot(autoTimer));
      stopShooting();
      intake.setGoal(Intake.Goal.INTAKE);
      co.await(sprintToPose(bump.getInitialPose().orElse(new Pose2d())).withTimeout(Seconds.of(3)));
      co.await(bump.cmd());
    });
    return routine;
  }

  /**
   * Builds a trench follow auto. Shoots preloaded ball first, then runs the follow trajectory
   * (intake at "Intake", shooter spinup at "Spin"), then aims and shoots while staging, then
   * gathers.
   */
  private AutoRoutine buildTrenchFollow(String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> followFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> gatherFactory,
      DoubleSupplier stagingX, DoubleSupplier stagingY) {
    AutoRoutine routine = newRoutine(name);
    AutoTrajectory follow = followFactory.apply(routine);
    AutoTrajectory gather = gatherFactory.apply(routine);
    bindIntakeAndSpin(follow);
    // Register once when building the routine, not on every command execution.
    Command gatherCommand = trenchGatherRun(gather);
    routine.run(co -> {
      co.await(follow.resetOdometry());
      co.await(shootFirstPreloadCommand());
      shooter.setHoodAngle(26.0);
      co.wait(Seconds.of(1));
      co.await(follow.cmd());
      intake.setGoal(Intake.Goal.IDLE);

      boolean red = ChoreoAllianceFlipUtil.shouldFlip();
      Translation2d staging = new Translation2d(stagingX.getAsDouble(), stagingY.getAsDouble());
      if (red) staging = ChoreoAllianceFlipUtil.flip(staging);
      Translation2d toHub = FieldConstants.getHubCenter(red).minus(staging);
      double aimAngle = Math.atan2(toHub.getY(), toHub.getX()) + Math.PI
          + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
      co.await(sprintToPose(new Pose2d(staging, new Rotation2d(aimAngle))).withTimeout(Seconds.of(2)));
      co.await(staticShot(null));
      indexer.setGoal(Indexer.Goal.IDLE);
      shooter.setHoodAngle(26.0);
      co.wait(Seconds.of(1));
      co.await(sprintToPose(gather.getInitialPose().orElse(new Pose2d())).withTimeout(Seconds.of(3)));
      co.await(gatherCommand);
    });
    return routine;
  }

  /**
   * Builds a trench points auto. Runs the points trajectory (intake deploys at "Intake" waypoint,
   * shooter spins up at "Spin" waypoint), then aims and shoots, then gathers until auto ends.
   */
  private AutoRoutine buildTrenchPoints(String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> pointsFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> bumpFactory) {
    AutoRoutine routine = newRoutine(name);
    AutoTrajectory path = pointsFactory.apply(routine);
    AutoTrajectory bump = bumpFactory.apply(routine);
    bindIntakeAndSpin(path);
    routine.run(co -> {
      Timer autoTimer = Timer.createStarted();
      co.await(path.resetOdometry());
      co.await(path.cmd());
      intake.setGoal(Intake.Goal.IDLE);
      co.await(staticShot(autoTimer));
      stopShooting();
      intake.setGoal(Intake.Goal.INTAKE);
      co.await(sprintToPose(bump.getInitialPose().orElse(new Pose2d())).withTimeout(Seconds.of(3)));
      co.await(bump.cmd());
    });
    return routine;
  }

  /**
   * Runs the gather trajectory once. Intake deploys at its "Intake" waypoint marker. Reactively
   * caps drive speed to Auto/GatherSlowSpeedMps when roller current exceeds
   * Auto/GatherClumpCurrentAmps, allowing the robot to push through dense ball packs without
   * shoving them away.
   */
  private Command trenchGatherRun(AutoTrajectory gather) {
    gather.atTime("Intake").onTrue(Command.noRequirements(co -> intake.setGoal(Intake.Goal.INTAKE)).named("Deploy Intake"));
    Runnable cleanup = drive::clearTrajectorySpeedCap;
    return Command.noRequirements(co -> {
      co.fork(Command.noRequirements(cap -> {
        while (true) {
          if (intake.getRollerStatorCurrentAmps() > gatherClumpCurrentAmps.get()) {
            drive.setTrajectorySpeedCap(gatherSlowSpeedMps.get());
          } else {
            drive.clearTrajectorySpeedCap();
          }
          cap.yield();
        }
      }).whenCanceled(cleanup).named("Gather Current Limit"));
      co.await(gather.cmd());
      cleanup.run();
    }).whenCanceled(cleanup).named("Gather Pass");
  }

  private static final double kTrenchAimToleranceRad = Math.toRadians(4.5);

  /**
   * Builds a double pass auto routine. Crosses bump, intakes across the field via trajectory,
   * returns over bump, aims at hub, and feeds for remaining time. If shootFirst is true, shoots the
   * preloaded ball before crossing the bump.
   */
  private AutoRoutine buildDoublePass(String name, String bumpTrajName,
      String doublePassTrajName, String bumpReturnTrajName, boolean shootFirst) {
    AutoRoutine routine = newRoutine(name);
    AutoTrajectory bump = routine.trajectory(bumpTrajName);
    AutoTrajectory pass = routine.trajectory(doublePassTrajName);
    AutoTrajectory back = routine.trajectory(bumpReturnTrajName);
    routine.run(co -> {
      co.await(bump.resetOdometry());
      if (shootFirst) co.await(shootFirstPreloadCommand());
      intake.setGoal(Intake.Goal.INTAKE);
      co.await(bump.cmd());
      co.await(sprintToPose(pass.getInitialPose().orElse(new Pose2d())).withTimeout(Seconds.of(3)));
      co.await(pass.cmd());
      shooter.setGoal(Shooter.Goal.SHOOT);
      intake.setGoal(Intake.Goal.IDLE);
      co.await(sprintToPose(back.getInitialPose().orElse(new Pose2d())).withTimeout(Seconds.of(3)));
      co.await(back.cmd());
      co.await(sprintToPose(back.getFinalPose().orElse(new Pose2d())).withTimeout(Seconds.of(2)));
      // The shooting scope ends after its 1 s settle and 10 s rehome/feed phase.
      co.await(scoringPhase("Return Shot", shot -> {
        shot.fork(aimBackAtHubWithVisionCreep());
        shot.wait(Seconds.of(1));
        indexer.setGoal(Indexer.Goal.FEED);
        shot.await(intake.periodicAutoRehomeCommand().withTimeout(Seconds.of(10)));
      }));
      stopShooting();
      intake.setGoal(Intake.Goal.INTAKE);
      co.await(bump.cmd());
    });
    return routine;
  }

  /**
   * Builds a safe auto routine. Deploys intake at the start, follows the Safe trajectory, then
   * shoots for remaining time. If shootFirst is true, shoots the preloaded ball before starting.
   */
  private AutoRoutine buildSafe(String name, boolean shootFirst) {
    AutoRoutine routine = newRoutine(name);
    AutoTrajectory path = routine.trajectory("Safe");
    path.atTime("Shoot").onTrue(Command.noRequirements(co -> shooter.setGoal(Shooter.Goal.SHOOT)).named("Spin Up Shooter"));
    routine.run(co -> {
      co.await(path.resetOdometry());
      if (shootFirst) co.await(shootFirstPreloadCommand());
      intake.setGoal(Intake.Goal.INTAKE);
      co.await(path.cmd());
      co.await(scoringPhase("Safe Shot", shot -> {
        shot.await(aimBackAtHub().withTimeout(Seconds.of(1)));
        indexer.setGoal(Indexer.Goal.FEED);
        shot.await(intake.periodicAutoRehomeCommand().withTimeout(Seconds.of(10)));
      }));
      stopShooting();
      intake.setGoal(Intake.Goal.IDLE);
    });
    return routine;
  }

  /** Sprint to a target pose at high speed with heading control. */
  private Command sprintToPose(Pose2d target) {
    return sprintToPose(target, 0.3);
  }

  /**
   * Sprint to a target pose. Exits when within {@code exitDistanceMeters} of the target, allowing a
   * larger value to hand off to a trajectory while still at speed.
   */
  private Command sprintToPose(Pose2d target, double exitDistanceMeters) {
    return drive.run(co -> {
      var heading = newHeadingController();
      do {
        Pose2d current = drive.getPose();
        Translation2d toTarget = target.getTranslation().minus(current.getTranslation());
        double distance = toTarget.getNorm();
        Rotation2d direction = toTarget.getAngle().orElse(Rotation2d.ZERO);
        double speed = Math.min(drive.getMaxLinearSpeedMetersPerSec(), Math.max(0.5, distance * 3));
        double omega = heading.calculate(current.getRotation().getRadians(), target.getRotation().getRadians());
        drive.runVelocity(new ChassisVelocities(direction.getCos() * speed,
            direction.getSin() * speed, omega).toRobotRelative(current.getRotation()));
        co.yield();
      } while (drive.getPose().getTranslation().getDistance(target.getTranslation()) >= exitDistanceMeters);
    }).whenCanceled(drive::stop).named("Sprint To Pose");
  }

  private Command driveTowardDepotAimingAtHub(double speedMps) {
    return drive.run(co -> {
      var heading = newHeadingController();
      while (true) {
        Pose2d current = drive.getPose();
        Translation2d toDepot = FieldConstants.getDepotApproachTarget(isRed()).minus(current.getTranslation());
        double angle = Math.atan2(toDepot.getY(), toDepot.getX());
        double omega = heading.calculate(current.getRotation().getRadians(), hubHeading());
        drive.runVelocity(new ChassisVelocities(Math.cos(angle) * speedMps,
            Math.sin(angle) * speedMps, omega).toRobotRelative(current.getRotation()));
        co.yield();
      }
    }).whenCanceled(drive::stop).named("Moving Shot Drive");
  }

  private Command aimBackAtHub() { return aimBackAtHub(false); }
  private Command aimBackAtHubWithVisionCreep() { return aimBackAtHub(true); }

  private Command aimBackAtHub(boolean creep) {
    return drive.run(co -> {
      var heading = newHeadingController();
      boolean tagsSeenOnce = false;
      while (true) {
        tagsSeenOnce |= vision.hasTagsInView() || !vision.hasCameraConnected();
        double vx = creep && !tagsSeenOnce
            ? (isRed() ? bumpCreepSpeedMps.get() : -bumpCreepSpeedMps.get()) : 0;
        double omega = heading.calculate(drive.getRotation().getRadians(), hubHeading());
        drive.runVelocity(new ChassisVelocities(vx, 0, omega).toRobotRelative(drive.getRotation()));
        co.yield();
      }
    }).whenCanceled(drive::stop).named(creep ? "Aim With Vision Creep" : "Aim At Hub");
  }

  private ProfiledPIDController newHeadingController() {
    var controller = new ProfiledPIDController(5.0, 0, 0.4,
        new TrapezoidProfile.Constraints(8.0, 20.0));
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.reset(drive.getRotation().getRadians());
    return controller;
  }

  private boolean isRed() { return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED; }

  private double hubHeading() {
    Translation2d toHub = FieldConstants.getHubCenter(isRed()).minus(drive.getPose().getTranslation());
    return Math.atan2(toHub.getY(), toHub.getX()) + Math.PI
        + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
  }

  /** Returns true when the back of the robot is aimed at the hub within kTrenchAimToleranceRad. */
  private boolean isAimedAtHub() {
    boolean isRed =
        MatchState.getAlliance().isPresent()
            && MatchState.getAlliance().get() == Alliance.RED;
    Translation2d hubCenter = FieldConstants.getHubCenter(isRed);
    Translation2d toHub = hubCenter.minus(drive.getPose().getTranslation());
    double targetHeading =
        MathUtil.angleModulus(
            Math.atan2(toHub.getY(), toHub.getX())
                + Math.PI
                + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees));
    double error = MathUtil.angleModulus(targetHeading - drive.getRotation().getRadians());
    return Math.abs(error) < kTrenchAimToleranceRad;
  }

  /** Reads the autonomous tunable and records the consumed delay as telemetry. */
  private double readShootFirstDelaySecs() {
    double delay = shootFirstDelaySecs.get();
    Logger.recordOutput("Auto/ShootFirstDelaySecsUsed", delay);
    org.wpilib.telemetry.Telemetry.log("Autonomous/ShootFirstDelaySecsUsed", delay);
    return delay;
  }

  /**
   * Shoots the preloaded ball and runs the minimum-delay timer in parallel. Proceeds when both are
   * done — so a delay shorter than the shoot adds no extra time, but a longer delay holds the robot
   * until it elapses.
   */
  private Command shootFirstPreloadCommand() {
    return Command.noRequirements(co -> {
      double minimumDelay = readShootFirstDelaySecs();
      Timer elapsed = Timer.createStarted();
      co.await(scoringPhase("Preload Feed", shot -> {
        shooter.setGoal(Shooter.Goal.SHOOT);
        shot.await(aimBackAtHub().withTimeout(Seconds.of(1.5)));
        indexer.setGoal(Indexer.Goal.FEED);
        shot.wait(Seconds.of(1));
      }));
      stopShooting();
      // Delay and shot share one start time: never add the full delay after the shot.
      co.waitUntil(() -> elapsed.hasElapsed(minimumDelay));
    }).whenCanceled(this::stopShooting).named("Preload Shot");
  }

  private AutoRoutine newRoutine(String name) {
    AutoRoutine routine = factory.newRoutine(name);
    routine.onCancel(() -> {
      stopShooting();
      intake.setGoal(Intake.Goal.IDLE);
      drive.clearTrajectorySpeedCap();
      drive.stop();
    });
    return routine;
  }

  private boolean canScore() {
    return shooter.isAvailable() && indexer.isAvailable();
  }

  /** Skip absent scoring hardware and cancel all scoped children if feedback is lost mid-shot. */
  private Command scoringPhase(String name, Consumer<Coroutine> body) {
    return Command.noRequirements(co -> {
      boolean available = canScore();
      Logger.recordOutput("Auto/ScoringPhaseSkipped", !available);
      if (available) {
        co.await(Command.noRequirements(body).until(() -> !canScore()).named(name));
      }
      if (!canScore()) stopShooting();
    }).whenCanceled(this::stopShooting).named(name + " With Availability");
  }

  private void stopShooting() {
    shooter.setGoal(Shooter.Goal.IDLE);
    indexer.setGoal(Indexer.Goal.IDLE);
  }

  private void bindIntakeAndSpin(AutoTrajectory path) {
    path.atTime("Intake").onTrue(Command.noRequirements(co -> intake.setGoal(Intake.Goal.INTAKE)).named("Deploy Intake"));
    path.atTime("Spin").onTrue(Command.noRequirements(co -> shooter.setGoal(Shooter.Goal.SHOOT)).named("Spin Up Shooter"));
  }

  /** A scope whose aiming/rehome children end as soon as its shooting timeline returns. */
  private Command staticShot(Timer autoTimer) {
    return scoringPhase("Static Shot", co -> {
      co.fork(aimBackAtHubWithVisionCreep(), intake.periodicAutoRehomeCommand());
      Debouncer aimed = new Debouncer(0.5, DebounceType.RISING);
      var readiness = co.waitUntil(() -> aimed.calculate(isAimedAtHub()), Seconds.of(2));
      // Preserve the original policy: after the bounded aiming wait, feed even on timeout.
      Logger.recordOutput("Auto/AimWaitTimedOut", readiness.timedOut());
      indexer.setGoal(Indexer.Goal.FEED);
      co.wait(Seconds.of(shootDurationSecs.get()));
      if (autoTimer != null) co.waitUntil(() -> autoTimer.get() >= bumpRushAutoTimeSecs.get());
    });
  }
}
