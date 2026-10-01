package frc.robot;

import frc.robot.util.V3Commands;

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
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Tunables;
// Migration pending: Choreo Alpha 3's autonomous APIs use Commands v2,
// while this project's command code currently uses Commands v3.
import org.wpilib.command3.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.Set;
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

  static {
    // setDefaultDouble only writes if the entry doesn't already exist, so Elastic values
    // survive robot restarts (unlike putNumber which always resets).
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getEntry("Auto/ShootFirstDelaySecs")
        .setDefaultDouble(0.0);
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
    AutoRoutine routine = factory.newRoutine("ResetOdometryStart");

    routine
        .active()
        .onTrue(
            factory
                .resetOdometry(() -> Optional.of(new Pose2d(3.559, 4.0296, Rotation2d.ZERO)))
                .andThen(V3Commands.print("Odometry reset to start position")).withAutomaticName());

    return routine;
  }

  public AutoRoutine depotCycle() {
    AutoRoutine routine = factory.newRoutine("Depot Cycle");

    // Load the routine's trajectories
    AutoTrajectory depotCycle = routine.trajectory("DepotCycle");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(V3Commands.sequence(depotCycle.resetOdometry(), depotCycle.cmd()));

    return routine;
  }

  public AutoRoutine depotInside() {
    AutoRoutine routine = factory.newRoutine("Depot Inside");

    // Load the routine's trajectories
    AutoTrajectory depotCycle = routine.trajectory("DepotInside");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(V3Commands.sequence(depotCycle.resetOdometry(), depotCycle.cmd()));

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
    AutoRoutine routine = factory.newRoutine("Depot Single Pass Shoot On Move");
    AutoTrajectory bump = routine.trajectory("DepotBump");
    AutoTrajectory singlePass = routine.trajectory("DepotSinglePass");
    AutoTrajectory bumpReturn = routine.trajectory("DepotBumpReturn");

    routine
        .active()
        .onTrue(
            V3Commands.sequence(
                // Seed odometry from bump trajectory start
                bump.resetOdometry(),

                // Deploy intake before crossing bump
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),

                // Cross the bump via Choreo trajectory
                bump.cmd(),

                // PID to single pass trajectory start
                sprintToPose(singlePass.getInitialPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(3.0)),

                // Run single pass trajectory with intake collecting balls
                singlePass.cmd(),

                // Retract intake, spin up shooter, and return over the bump.
                // The shooter has the full bump-return transit time to reach setpoint.
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),
                V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                bumpReturn.cmd(),

                // Correct positional error introduced by bump crossing (same as Shoot First auto)
                sprintToPose(bumpReturn.getFinalPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(2.0)),

                // Now on the alliance side: drive toward depot at 0.5 m/s while shooting.
                // Three branches run in parallel until auto ends:
                //   1. Drive + heading: translate toward depot approach target, back aimed at hub
                //   2. Feed: wait for shooter setpoint, then own the indexer in FEED mode
                //   3. Intake: jostle balls (periodic rehome) until within ~2m of the depot,
                //      then deploy to collect staged balls -- mutually exclusive with jostling
                V3Commands.parallel(
                    driveTowardDepotAimingAtHub(0.5),
                    V3Commands.sequence(
                        V3Commands.waitUntil(() -> shooter.isAtSetpoint()), indexer.feedCommand()),
                    V3Commands.sequence(
                        intake
                            .periodicAutoRehomeCommand()
                            .until(
                                () -> {
                                  boolean isRed =
                                      MatchState.getAlliance().isPresent()
                                          && MatchState.getAlliance().get() == Alliance.RED;
                                  return drive
                                          .getPose()
                                          .getTranslation()
                                          .getDistance(FieldConstants.getDepotCenter(isRed))
                                      < 2.0;
                                }).withAutomaticName(),
                        V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE))))));

    return routine;
  }

  public AutoRoutine safe() {
    return buildSafe("Safe", false);
  }

  public AutoRoutine safeShootFirst() {
    return buildSafe("Safe Shoot First", true);
  }

  public AutoRoutine shootOnly() {
    AutoRoutine routine = factory.newRoutine("Shoot Only");

    routine
        .active()
        .onTrue(
            V3Commands.sequence(
                V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                aimBackAtHub().withTimeout(org.wpilib.units.Units.Seconds.of(1.5)),
                V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                intake.periodicAutoRehomeCommand().withTimeout(org.wpilib.units.Units.Seconds.of(10.0)),
                V3Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                    })));

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
  private AutoRoutine buildTrenchDisrupt(
      String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> disruptFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> bumpFactory) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory disruptTraj = disruptFactory.apply(routine);
    AutoTrajectory bumpTraj = bumpFactory.apply(routine);
    Timer autoTimer = new Timer();

    // Deploy intake at the "Intake" waypoint on the disrupt trajectory
    disruptTraj.atTime("Intake").onTrue(V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    // Spin up shooter at the "Spin" waypoint (before crossing bump)
    disruptTraj.atTime("Spin").onTrue(V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            V3Commands.sequence(
                V3Commands.runOnce(() -> autoTimer.restart()),
                disruptTraj.resetOdometry(),

                // Rush out; intake and shooter activate via waypoint events
                disruptTraj.cmd(),

                // Retract intake for shooting
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // Static shot: wait until aimed for 0.5s, feed for at least shootDurationSecs,
                // then keep shooting until bumpRushAutoTimeSecs have elapsed since routine start.
                V3Commands.deadline(
                    V3Commands.sequence(
                        V3Commands.defer(
                            () -> {
                              Debouncer aimDebouncer = new Debouncer(0.5, DebounceType.RISING);
                              return V3Commands.waitUntil(
                                      () -> aimDebouncer.calculate(isAimedAtHub()))
                                  .withTimeout(org.wpilib.units.Units.Seconds.of(2.0));
                            },
                            Set.of()),
                        V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        V3Commands.defer(
                            () -> V3Commands.waitSeconds(shootDurationSecs.get()), Set.of()),
                        V3Commands.waitUntil(() -> autoTimer.get() >= bumpRushAutoTimeSecs.get())),
                    aimBackAtHubWithVisionCreep(),
                    intake.periodicAutoRehomeCommand()),

                // Stop shooter and indexer
                V3Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                    }),

                // Rush over bump with intake deployed
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                sprintToPose(bumpTraj.getInitialPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(3.0)),
                bumpTraj.cmd()));

    return routine;
  }

  /**
   * Builds a trench follow auto. Shoots preloaded ball first, then runs the follow trajectory
   * (intake at "Intake", shooter spinup at "Spin"), then aims and shoots while staging, then
   * gathers.
   */
  private AutoRoutine buildTrenchFollow(
      String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> followFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> gatherFactory,
      DoubleSupplier stagingX,
      DoubleSupplier stagingY) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory followTraj = followFactory.apply(routine);
    AutoTrajectory gatherTraj = gatherFactory.apply(routine);

    followTraj.atTime("Intake").onTrue(V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    followTraj.atTime("Spin").onTrue(V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            V3Commands.sequence(
                followTraj.resetOdometry(),

                // Shoot preloaded ball before driving out; delay runs in parallel so it doesn't
                // add time if the shot itself takes longer.
                shootFirstPreloadCommand(),

                // Lower hood to 26° before entering trench (first pass)
                V3Commands.runOnce(() -> shooter.setHoodAngle(26.0)),
                V3Commands.waitSeconds(1.0),

                // Follow trajectory; intake and shooter activate via waypoint events
                followTraj.cmd(),

                // Retract intake for shooting
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // Sprint to staging position with back of robot aimed at hub
                V3Commands.defer(
                    () -> {
                      boolean isRed = ChoreoAllianceFlipUtil.shouldFlip();
                      Translation2d staging =
                          new Translation2d(stagingX.getAsDouble(), stagingY.getAsDouble());
                      if (isRed) staging = ChoreoAllianceFlipUtil.flip(staging);
                      Translation2d hubCenter = FieldConstants.getHubCenter(isRed);
                      Translation2d toHub = hubCenter.minus(staging);
                      double aimAngle =
                          Math.atan2(toHub.getY(), toHub.getX())
                              + Math.PI
                              + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
                      return sprintToPose(new Pose2d(staging, new Rotation2d(aimAngle)))
                          .withTimeout(org.wpilib.units.Units.Seconds.of(2.0));
                    },
                    Set.of(drive)),

                // Static shoot from staging position
                V3Commands.deadline(
                    V3Commands.sequence(
                        V3Commands.defer(
                            () -> {
                              Debouncer aimDebouncer = new Debouncer(0.5, DebounceType.RISING);
                              return V3Commands.waitUntil(
                                      () -> aimDebouncer.calculate(isAimedAtHub()))
                                  .withTimeout(org.wpilib.units.Units.Seconds.of(2.0));
                            },
                            Set.of()),
                        V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        V3Commands.defer(
                            () -> V3Commands.waitSeconds(shootDurationSecs.get()), Set.of())),
                    aimBackAtHubWithVisionCreep(),
                    intake.periodicAutoRehomeCommand()),

                // Stop indexer before gather pass
                V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.IDLE)),

                // Lower hood to 26° before entering trench for gather pass
                V3Commands.runOnce(() -> shooter.setHoodAngle(26.0)),
                V3Commands.waitSeconds(1.0),

                // PID to gather path start so second pass is consistent
                sprintToPose(gatherTraj.getInitialPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(3.0)),

                // Gather: run OutpostStagingGather, deploying intake at its "Intake" marker
                trenchGatherRun(gatherTraj)));

    return routine;
  }

  /**
   * Builds a trench points auto. Runs the points trajectory (intake deploys at "Intake" waypoint,
   * shooter spins up at "Spin" waypoint), then aims and shoots, then gathers until auto ends.
   */
  private AutoRoutine buildTrenchPoints(
      String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> pointsFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> bumpFactory) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory pointsTraj = pointsFactory.apply(routine);
    AutoTrajectory bumpTraj = bumpFactory.apply(routine);
    Timer autoTimer = new Timer();

    // Deploy intake at the "Intake" waypoint marker
    pointsTraj.atTime("Intake").onTrue(V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));

    // Spin up shooter at the "Spin" waypoint (before crossing bump)
    pointsTraj.atTime("Spin").onTrue(V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            V3Commands.sequence(
                V3Commands.runOnce(() -> autoTimer.restart()),
                pointsTraj.resetOdometry(),

                // Follow Points trajectory; intake deploys at "Intake", shooter spins at "Spin"
                pointsTraj.cmd(),

                // Retract intake for static shot
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // Static shot: wait until aimed for 0.5s, feed for at least shootDurationSecs,
                // then keep shooting until bumpRushAutoTimeSecs have elapsed since routine start.
                V3Commands.deadline(
                    V3Commands.sequence(
                        V3Commands.defer(
                            () -> {
                              Debouncer aimDebouncer = new Debouncer(0.5, DebounceType.RISING);
                              return V3Commands.waitUntil(
                                      () -> aimDebouncer.calculate(isAimedAtHub()))
                                  .withTimeout(org.wpilib.units.Units.Seconds.of(2.0));
                            },
                            Set.of()),
                        V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        V3Commands.defer(
                            () -> V3Commands.waitSeconds(shootDurationSecs.get()), Set.of()),
                        V3Commands.waitUntil(() -> autoTimer.get() >= bumpRushAutoTimeSecs.get())),
                    aimBackAtHubWithVisionCreep(),
                    intake.periodicAutoRehomeCommand()),

                // Stop shooter and indexer
                V3Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                    }),

                // Rush over bump with intake deployed
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                sprintToPose(bumpTraj.getInitialPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(3.0)),
                bumpTraj.cmd()));

    return routine;
  }

  /**
   * Runs the gather trajectory once. Intake deploys at its "Intake" waypoint marker. Reactively
   * caps drive speed to Auto/GatherSlowSpeedMps when roller current exceeds
   * Auto/GatherClumpCurrentAmps, allowing the robot to push through dense ball packs without
   * shoving them away.
   */
  private Command trenchGatherRun(AutoTrajectory gatherTraj) {
    gatherTraj.atTime("Intake").onTrue(V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    return V3Commands.finallyDo(V3Commands.deadline(
            gatherTraj.cmd(),
            V3Commands.run(
                () -> {
                  if (intake.getRollerStatorCurrentAmps() > gatherClumpCurrentAmps.get()) {
                    drive.setTrajectorySpeedCap(gatherSlowSpeedMps.get());
                  } else {
                    drive.clearTrajectorySpeedCap();
                  }
                })), interrupted -> drive.clearTrajectorySpeedCap());
  }

  private static final double kTrenchAimToleranceRad = Math.toRadians(4.5);

  /**
   * Builds a double pass auto routine. Crosses bump, intakes across the field via trajectory,
   * returns over bump, aims at hub, and feeds for remaining time. If shootFirst is true, shoots the
   * preloaded ball before crossing the bump.
   */
  private AutoRoutine buildDoublePass(
      String name,
      String bumpTrajName,
      String doublePassTrajName,
      String bumpReturnTrajName,
      boolean shootFirst) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory bump = routine.trajectory(bumpTrajName);
    AutoTrajectory doublePass = routine.trajectory(doublePassTrajName);
    AutoTrajectory bumpReturn = routine.trajectory(bumpReturnTrajName);

    // Build shoot-first prefix if needed
    Command shootFirstSequence = shootFirst ? shootFirstPreloadCommand() : V3Commands.none();

    routine
        .active()
        .onTrue(
            V3Commands.sequence(
                // Seed odometry from bump trajectory start
                bump.resetOdometry(),

                // Optionally shoot preloaded ball first
                shootFirstSequence,

                // Deploy intake before crossing bump
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),

                // Cross the bump via Choreo trajectory
                bump.cmd(),

                // PID to double pass trajectory start
                sprintToPose(doublePass.getInitialPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(3.0)),

                // Run double pass trajectory
                doublePass.cmd(),

                // Spin up shooter (auto-tracks RPM/hood angle from distance LUT)
                V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),

                // Retract intake before crossing back over the bump
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // PID to bump return trajectory start
                sprintToPose(bumpReturn.getInitialPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(3.0)),

                // Cross back over bump via Choreo trajectory
                bumpReturn.cmd(),

                // Sprint to final shooting position after bump (corrects positional error from bump
                // crossing)
                sprintToPose(bumpReturn.getFinalPose().orElse(new Pose2d())).withTimeout(org.wpilib.units.Units.Seconds.of(2.0)),

                // Aim at hub while feeding/shooting. If no tags are visible (odometry may have
                // drifted over the bump), creep toward alliance wall until vision is restored.
                V3Commands.deadline(
                    V3Commands.sequence(
                        V3Commands.waitSeconds(1.0),
                        V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        intake.periodicAutoRehomeCommand().withTimeout(org.wpilib.units.Units.Seconds.of(10.0))),
                    aimBackAtHubWithVisionCreep()),

                // Stop shooter and indexer, then rush back over the bump toward the middle of the
                // field with intake deployed to pick up any staged balls along the way
                V3Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                    }),
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                bump.cmd()));

    return routine;
  }

  /**
   * Builds a safe auto routine. Deploys intake at the start, follows the Safe trajectory, then
   * shoots for remaining time. If shootFirst is true, shoots the preloaded ball before starting.
   */
  private AutoRoutine buildSafe(String name, boolean shootFirst) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory safeTraj = routine.trajectory("Safe");

    // Build shoot-first prefix if needed
    Command shootFirstSequence = shootFirst ? shootFirstPreloadCommand() : V3Commands.none();

    // Spin up shooter on the "Shoot" event marker during trajectory
    safeTraj.atTime("Shoot").onTrue(V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            V3Commands.sequence(
                // Seed odometry from trajectory start
                safeTraj.resetOdometry(),

                // Optionally shoot preloaded ball first
                shootFirstSequence,

                // Deploy intake at beginning of trajectory
                V3Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),

                // Follow the Safe trajectory
                safeTraj.cmd(),

                // Fine-tune aim (back at hub) before shooting
                aimBackAtHub().withTimeout(org.wpilib.units.Units.Seconds.of(1.0)),

                // Feed and shoot for remaining time
                V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                intake.periodicAutoRehomeCommand().withTimeout(org.wpilib.units.Units.Seconds.of(10.0)),

                // Cleanup
                V3Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                      intake.setGoal(Intake.Goal.IDLE);
                    })));

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
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return V3Commands.beforeStarting(V3Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              Translation2d toTarget = target.getTranslation().minus(current.getTranslation());
              double distance = toTarget.getNorm();
              Rotation2d direction = toTarget.getAngle().orElse(Rotation2d.ZERO);

              // Full speed, decelerate in last 0.5m
              double speed =
                  Math.min(drive.getMaxLinearSpeedMetersPerSec(), Math.max(0.5, distance * 3.0));

              double omega =
                  headingController.calculate(
                      current.getRotation().getRadians(), target.getRotation().getRadians());

              drive.runVelocity(
                  new ChassisVelocities(direction.getCos() * speed, direction.getSin() * speed, omega).toRobotRelative(current.getRotation()));
            },
            drive)
        , () -> headingController.reset(drive.getRotation().getRadians()))
        .until(
            () ->
                drive.getPose().getTranslation().getDistance(target.getTranslation())
                    < exitDistanceMeters).withAutomaticName();
  }

  /**
   * Translate toward the depot-side fuel pool center at {@code speedMps} while the heading PID
   * independently keeps the back of the robot aimed at the hub. Translation and rotation are
   * decoupled, so shoot-on-move compensation in the shooter handles the moving shot correctly.
   */
  private Command driveTowardDepotAimingAtHub(double speedMps) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return V3Commands.beforeStarting(V3Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              boolean isRed =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;

              // Rotation: keep back of robot aimed at hub
              Translation2d hubCenter = FieldConstants.getHubCenter(isRed);
              Translation2d toHub = hubCenter.minus(current.getTranslation());
              double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
              double targetHeading =
                  angleToHub
                      + Math.PI
                      + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              // Translation: drive toward depot
              Translation2d depotTarget = FieldConstants.getDepotApproachTarget(isRed);
              Translation2d toDepot = depotTarget.minus(current.getTranslation());
              double driveAngle = Math.atan2(toDepot.getY(), toDepot.getX());

              drive.runVelocity(
                  new ChassisVelocities(Math.cos(driveAngle) * speedMps, Math.sin(driveAngle) * speedMps, omega).toRobotRelative(current.getRotation()));
            },
            drive)
        , () -> headingController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Drive to {@code target} in two phases while aiming the back of the robot at the hub: first
   * travel in the Y direction until within 0.3m of the target Y, then travel in the X direction
   * until within 0.3m of the target overall. Decelerates proportionally as in sprintToPose.
   */
  private Command driveYThenXAimingAtHub(Pose2d target) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return V3Commands.beforeStarting(V3Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              boolean isRed =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;

              // Heading: keep back of robot aimed at hub
              Translation2d hubCenter = FieldConstants.getHubCenter(isRed);
              Translation2d toHub = hubCenter.minus(current.getTranslation());
              double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
              double targetHeading =
                  angleToHub
                      + Math.PI
                      + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              // Translation: Y-first then X
              double yError = target.getY() - current.getY();
              double xError = target.getX() - current.getX();

              double vx, vy;
              if (Math.abs(yError) > 0.3) {
                // Phase 1: drive in Y direction
                double speed =
                    Math.min(
                        drive.getMaxLinearSpeedMetersPerSec(),
                        Math.max(0.5, Math.abs(yError) * 3.0));
                vy = Math.signum(yError) * speed;
                vx = 0;
              } else {
                // Phase 2: drive in X direction
                double speed =
                    Math.min(
                        drive.getMaxLinearSpeedMetersPerSec(),
                        Math.max(0.5, Math.abs(xError) * 3.0));
                vx = Math.signum(xError) * speed;
                vy = 0;
              }

              drive.runVelocity(
                  new ChassisVelocities(vx, vy, omega).toRobotRelative(current.getRotation()));
            },
            drive)
        , () -> headingController.reset(drive.getRotation().getRadians()))
        .until(() -> drive.getPose().getTranslation().getDistance(target.getTranslation()) < 0.3).withAutomaticName();
  }

  /** Rotate in place to aim back of robot at hub. */
  private Command aimBackAtHub() {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return V3Commands.beforeStarting(V3Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              boolean isRed =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;
              Translation2d hubCenter = FieldConstants.getHubCenter(isRed);

              Translation2d toHub = hubCenter.minus(current.getTranslation());
              double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
              double targetHeading =
                  angleToHub
                      + Math.PI
                      + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);

              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              drive.runVelocity(
                  new ChassisVelocities(0, 0, omega).toRobotRelative(current.getRotation()));
            },
            drive)
        , () -> headingController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Like aimBackAtHub(), but also creeps toward the alliance wall when no AprilTags are visible.
   * Used after returning over the bump, where odometry drift can leave the robot too close to the
   * hub to see tags. The creep moves it into range; vision then corrects the pose estimate.
   */
  private Command aimBackAtHubWithVisionCreep() {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    // Latches true the first time tags are seen after the bump return. Once vision has had one
    // good look it has corrected odometry, so we never creep again for this shoot phase.
    boolean[] tagsSeenOnce = {false};

    return V3Commands.beforeStarting(V3Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              boolean isRed =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;
              Translation2d hubCenter = FieldConstants.getHubCenter(isRed);

              Translation2d toHub = hubCenter.minus(current.getTranslation());
              double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
              double targetHeading =
                  angleToHub
                      + Math.PI
                      + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);

              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              if (vision.hasTagsInView() || !vision.hasCameraConnected()) {
                tagsSeenOnce[0] = true;
              }
              double vx = 0;
              if (!tagsSeenOnce[0]) {
                vx = isRed ? bumpCreepSpeedMps.get() : -bumpCreepSpeedMps.get();
              }

              drive.runVelocity(
                  new ChassisVelocities(vx, 0, omega).toRobotRelative(current.getRotation()));
            },
            drive)
        , () -> headingController.reset(drive.getRotation().getRadians()));
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

  /**
   * Reads Auto/ShootFirstDelaySecs from SmartDashboard and logs the value both to AdvantageKit and
   * SmartDashboard so it is visible in Elastic at the moment it is consumed.
   */
  private double readShootFirstDelaySecs() {
    double delay = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Auto/ShootFirstDelaySecs").getDouble(0.0);
    Logger.recordOutput("Auto/ShootFirstDelaySecsUsed", delay);
    NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Auto/ShootFirstDelaySecsUsed").setDouble(delay);
    return delay;
  }

  /**
   * Shoots the preloaded ball and runs the minimum-delay timer in parallel. Proceeds when both are
   * done — so a delay shorter than the shoot adds no extra time, but a longer delay holds the robot
   * until it elapses.
   */
  private Command shootFirstPreloadCommand() {
    return V3Commands.parallel(
        V3Commands.sequence(
            V3Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
            aimBackAtHub().withTimeout(org.wpilib.units.Units.Seconds.of(1.5)),
            V3Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
            V3Commands.waitSeconds(1.0),
            V3Commands.runOnce(
                () -> {
                  indexer.setGoal(Indexer.Goal.IDLE);
                  shooter.setGoal(Shooter.Goal.IDLE);
                })),
        V3Commands.defer(() -> V3Commands.waitSeconds(readShootFirstDelaySecs()), Set.of()));
  }
}
