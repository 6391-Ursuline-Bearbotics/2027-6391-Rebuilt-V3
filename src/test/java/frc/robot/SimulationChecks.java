package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.SteppedRobot;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArraySubscriber;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.NiDsXboxControllerSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.Timer;
import org.wpilib.system.RobotController;
import org.wpilib.util.WPIUtilJNI;

/** Physics IO simulation, driver-station lifecycle, joystick bindings, chooser and Field2D checks. */
public final class SimulationChecks implements AutoCloseable {
  private final SteppedRobot robot;
  private final RobotContainer container;
  private final NiDsXboxControllerSim joystick;
  private final boolean playback;
  private final StructArraySubscriber<Pose2d> fieldPoses;
  private final Map<Path, byte[]> savedPreferences = new HashMap<>();
  private String scenario = "initialization";
  private boolean redAlliance;
  private final List<String> rows = new ArrayList<>(List.of(
      "scenario,alliance,time,actual_x,actual_y,actual_heading,expected_x,expected_y,expected_heading,error_m"));

  private SimulationChecks(boolean playback) { this(playback, false); }

  private SimulationChecks(boolean playback, boolean missingMechanisms) {
    this.playback = playback;
    if (!playback) {
      for (String name : List.of("networktables.json", "networktables.json.bck")) {
        Path path = Path.of(name);
        try { savedPreferences.put(path, Files.exists(path) ? Files.readAllBytes(path) : null); }
        catch (IOException e) { throw new RuntimeException(e); }
      }
    }
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    // PhotonCameraSim schedules frames on the NT clock; advance it with HAL, not wall time.
    WPIUtilJNI.enableMockTime();
    WPIUtilJNI.setMockTime(RobotController.getMonotonicTime());
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setSendError(playback);
    robot = new SteppedRobot(() -> new RobotContainer(missingMechanisms));
    container = ((Robot) robot).getContainer();
    if (!playback) {
      // Headless checks must not save their temporary tuning values as operator preferences.
      NetworkTableInstance.getDefault()
          .getTopic("/Tunables/Autonomous/ShootFirstDelaySecs/value").setPersistent(false);
    }
    fieldPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/Telemetry/Drive/Field/Robot", Pose2d.struct)
        .subscribe(new Pose2d[0]);
    joystick = new NiDsXboxControllerSim(0);
    joystick.setAxesAvailable(0x3f);
    joystick.setButtonsAvailable(0x3ff);
    new NiDsXboxControllerSim(1).setAxesAvailable(0x3f);
    robot.simulationInit();
    DriverStationBackend.observeUserProgramStarting();
    SimHooks.setProgramStarted(true);
    if (playback) {
      tick();
      try { Thread.sleep(15_000); }
      catch (InterruptedException e) { Thread.currentThread().interrupt(); throw new RuntimeException(e); }
    }
  }

  private void tick() {
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.02);
    WPIUtilJNI.setMockTime(RobotController.getMonotonicTime());
    robot.step();
    fieldMatchesPose();
    if (scenario.equals("Teleop")) recordPose(Timer.getTimestamp(), null);
    if (playback) {
      try { Thread.sleep(20); }
      catch (InterruptedException e) { Thread.currentThread().interrupt(); throw new RuntimeException(e); }
    }
  }

  private void ticks(double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) tick();
  }

  private void mode(RobotMode mode, boolean enabled, boolean red) {
    DriverStationSim.setAllianceStationId(red ? AllianceStationID.RED_1 : AllianceStationID.BLUE_1);
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.setOpMode((long) mode.getValue() << 56);
    DriverStationSim.setEnabled(enabled);
    tick();
  }

  private void neutral() {
    joystick.setLeftX(0);
    joystick.setLeftY(0);
    joystick.setRightX(0);
  }

  private void fieldMatchesPose() {
    var nt = NetworkTableInstance.getDefault();
    assertEquals("Field2d", nt.getEntry("/Telemetry/Drive/Field/.type").getString(""));
    Pose2d[] published = fieldPoses.get();
    assertEquals(1, published.length, "Field2D must publish one robot pose");
    assertEquals(container.getDrive().getPose().getX(), published[0].getX(), 1e-6);
    assertEquals(container.getDrive().getPose().getY(), published[0].getY(), 1e-6);
    assertEquals(0, container.getDrive().getRotation().minus(published[0].getRotation())
        .getRadians(), 1e-6);
  }

  private void teleop(boolean red) {
    scenario = "initialization";
    redAlliance = red;
    neutral();
    mode(RobotMode.TELEOPERATED, false, red);
    container.getDrive().setPose(new Pose2d(8, 4, Rotation2d.ZERO));
    scenario = "Teleop";
    mode(RobotMode.TELEOPERATED, true, red);
    double sign = red ? -1 : 1;
    Pose2d start = container.getDrive().getPose();
    joystick.setLeftY(-0.6);
    ticks(1.5);
    Pose2d forward = container.getDrive().getPose();
    double distance = sign * (forward.getX() - start.getX());
    assertTrue(distance > 0.8 && distance < 4, "Forward travel: " + distance);
    assertEquals(start.getY(), forward.getY(), 0.15, "Forward lateral drift");
    neutral();
    ticks(0.6);
    Pose2d stopped = container.getDrive().getPose();
    ticks(0.6);
    assertTrue(stopped.getTranslation().getDistance(container.getDrive().getPose().getTranslation())
        < 0.12, "Joystick release must stop the drive");
    joystick.setLeftX(-0.6);
    ticks(1.5);
    Pose2d strafe = container.getDrive().getPose();
    assertTrue(sign * (strafe.getY() - stopped.getY()) > 0.8,
        "Strafe direction: from " + stopped + " to " + strafe);
    neutral();
    ticks(0.6);
    double heading = container.getDrive().getRotation().getRadians();
    joystick.setRightX(-0.4);
    ticks(1.0);
    assertTrue(container.getDrive().getRotation().minus(
        new Rotation2d(heading)).getRadians() > 0.25, "Rotation input");
    neutral();
    ticks(0.6);
    mode(RobotMode.TELEOPERATED, false, red);
    ticks(0.6);
    Pose2d disabled = container.getDrive().getPose();
    joystick.setLeftY(-0.8);
    ticks(1.0);
    assertTrue(disabled.getTranslation().getDistance(container.getDrive().getPose().getTranslation())
        < 0.05, "Disabled robot must ignore joystick input");
    neutral();
    fieldMatchesPose();
    System.out.printf("PASS teleop %s: forward %.2f m, strafe and turn, stop and disable%n",
        red ? "red" : "blue", distance);
  }

  private void auto(String name, String path, boolean mirrorY, boolean red) {
    scenario = name;
    redAlliance = red;
    neutral();
    mode(RobotMode.AUTONOMOUS, false, red);
    Scheduler.getDefault().cancelAll();
    var selected = NetworkTableInstance.getDefault().getEntry("/Tunables/Autonomous/Chooser/selected/tune");
    selected.setString(name);
    NetworkTableInstance.getDefault().flushLocal();
    NetworkTableInstance.getDefault().waitForListenerQueue(1.0);
    ticks(0.1);
    assertEquals(name, container.getSelectedAutoName(), "Dashboard auto selection");
    var trajectory = Choreo.<SwerveSample>loadTrajectory(path).orElseThrow();
    if (mirrorY) trajectory = trajectory.mirrorY();
    // Preview endpoints must agree with the selected routine, including alliance and side mirroring.
    try (var preview = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/Telemetry/Autonomous/Preview/path", Pose2d.struct)
        .subscribe(new Pose2d[0])) {
      Pose2d[] poses = preview.get();
      assertTrue(poses.length >= trajectory.samples().size(), "Preview must show the chosen path");
      assertEquals(0, poses[0].getTranslation().getDistance(
          trajectory.getInitialPose(red).orElseThrow().getTranslation()), 1e-6);
    }
    mode(RobotMode.AUTONOMOUS, true, red);
    double started = Timer.getTimestamp();
    double maxError = 0;
    double finalError = 0;
    double maxHeadingError = 0;
    double finalHeadingError = 0;
    for (int i = 0; i <= Math.ceil(trajectory.getTotalTime() / 0.02); i++) {
      double elapsed = Timer.getTimestamp() - started;
      Pose2d expected = trajectory.sampleAt(elapsed, red).orElseThrow().getPose();
      Pose2d actual = fieldPoses.get()[0];
      finalError = actual.getTranslation().getDistance(expected.getTranslation());
      maxError = Math.max(maxError, finalError);
      finalHeadingError = Math.abs(actual.getRotation().minus(expected.getRotation()).getDegrees());
      maxHeadingError = Math.max(maxHeadingError, finalHeadingError);
      recordPose(elapsed, expected);
      tick();
    }
    System.out.printf("AUTO %s %s: peak %.3f m, endpoint %.3f m%n", name,
        red ? "red" : "blue", maxError, finalError);
    // A 35 cm transient bound is under half the drivebase width; require 15 cm at the endpoint.
    assertTrue(maxError < 0.35, name + " peak tracking error " + maxError);
    assertTrue(finalError < 0.15, name + " endpoint error " + finalError);
    assertTrue(maxHeadingError < 20, name + " transient heading error " + maxHeadingError);
    assertTrue(finalHeadingError < 5, name + " endpoint heading error " + finalHeadingError);
    fieldMatchesPose();
    // Finish the Safe routine's bounded shooting phase as well.
    if (name.equals("Safe")) ticks(12);
    mode(RobotMode.TELEOPERATED, true, red);
    ticks(0.6);
    assertTrue(Math.hypot(container.getDrive().getChassisVelocities().vx,
        container.getDrive().getChassisVelocities().vy) < 0.15, "Auto-to-teleop must stop motion");
    // Restart, then disable mid-path. Commands must not resume on the next teleop entry.
    mode(RobotMode.AUTONOMOUS, true, red);
    ticks(0.3);
    mode(RobotMode.AUTONOMOUS, false, red);
    ticks(0.6);
    Pose2d disabled = container.getDrive().getPose();
    ticks(0.5);
    assertEquals(0, disabled.getTranslation().getDistance(
        container.getDrive().getPose().getTranslation()), 0.03, name + " " + red + " disabled auto must stop");
    mode(RobotMode.TELEOPERATED, true, red);
    ticks(0.5);
    assertEquals(0, disabled.getTranslation().getDistance(
        container.getDrive().getPose().getTranslation()), 0.05, "Canceled auto must stay canceled");
  }

  private void recordPose(double time, Pose2d expected) {
    Pose2d actual = fieldPoses.get()[0];
    rows.add(String.format(java.util.Locale.ROOT,
        "%s,%s,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",
        scenario, redAlliance ? "red" : "blue", time, actual.getX(), actual.getY(),
        actual.getRotation().getDegrees(), expected == null ? Double.NaN : expected.getX(),
        expected == null ? Double.NaN : expected.getY(),
        expected == null ? Double.NaN : expected.getRotation().getDegrees(),
        expected == null ? Double.NaN : actual.getTranslation().getDistance(expected.getTranslation())));
  }

  public static void run(boolean playback) throws Exception {
    var failures = new ArrayList<Throwable>();
    try (var checks = new SimulationChecks(playback)) {
      for (boolean red : new boolean[] {false, true}) {
        try { checks.teleop(red); } catch (AssertionError e) { failures.add(e); }
        for (String name : List.of("Depot Cycle", "Depot Inside", "Safe")) {
          try { checks.auto(name, name.replace(" ", ""), false, red); }
          catch (AssertionError e) { failures.add(e); }
        }
        try { checks.auto("Trench Depot Points", "TrenchOutpostPoints", true, red); }
        catch (AssertionError e) { failures.add(e); }
      }
      Files.createDirectories(Path.of("build/reports/simulation"));
      Files.write(Path.of("build/reports/simulation/field-poses.csv"), checks.rows);
    }
    assertAll("Simulator functionality", failures.stream().map(failure ->
        (org.junit.jupiter.api.function.Executable) () -> { throw failure; }));
  }


  /** The real lifecycle and drive physics must remain usable with every optional IO absent. */
  public static void runDriveOnly() throws Exception {
    try (var checks = new SimulationChecks(false, true)) {
      for (boolean red : new boolean[] {false, true}) {
        checks.teleop(red);
        for (String name : List.of("Depot Cycle", "Depot Inside", "Safe")) {
          checks.auto(name, name.replace(" ", ""), false, red);
        }
        checks.auto("Trench Depot Points", "TrenchOutpostPoints", true, red);
        checks.auto("Safe (Shoot First)", "Safe", false, red);
        // Check later paths too: skipping a score must not terminate the parent routine.
        for (var entry : Map.of(
            "Trench Depot Points", "Follow DepotBump",
            "Trench Depot Follow", "Follow OutpostStagingGather",
            "Depot Double Pass", "Follow DepotBump",
            "Depot Single Pass Shoot On Move", "Moving Shot Drive").entrySet()) {
          checks.mode(RobotMode.AUTONOMOUS, false, red);
          var nt = NetworkTableInstance.getDefault();
          nt.getEntry("/Tunables/Autonomous/Chooser/selected/tune").setString(entry.getKey());
          nt.flushLocal();
          nt.waitForListenerQueue(1);
          checks.ticks(0.1);
          checks.mode(RobotMode.AUTONOMOUS, true, red);
          int starts = 0;
          boolean previouslyRunning = false;
          for (int cycle = 0; cycle < 2000; cycle++) {
            checks.tick();
            assertFalse(checks.container.getShooter().isAvailable());
            assertFalse(checks.container.getShooter().isAtSetpoint());
            assertFalse(checks.container.getIntake().isAvailable());
            assertFalse(checks.container.getIndexer().isAvailable());
            assertNotEquals(frc.robot.subsystems.indexer.Indexer.Goal.FEED,
                checks.container.getIndexer().getGoal(), "Absent scoring hardware must never feed");
            boolean running = Scheduler.getDefault().getRunningCommands().stream()
                .anyMatch(command -> command.name().equals(entry.getValue()));
            if (running && !previouslyRunning) starts++;
            previouslyRunning = running;
          }
          int expectedStarts = entry.getKey().equals("Depot Double Pass") ? 2 : 1;
          assertTrue(starts >= expectedStarts, entry.getKey() + " must reach "
              + entry.getValue() + " " + expectedStarts + " time(s), saw " + starts);
          checks.mode(RobotMode.AUTONOMOUS, false, red);
          checks.ticks(0.2);
          assertTrue(Scheduler.getDefault().getRunningCommands().stream()
              .noneMatch(command -> command.name().contains("Moving Shot Drive")
                  || command.name().contains("Aim With Vision Creep")));
        }
        // Held shot buttons and hood positioning cannot seize manual drive on a bare chassis.
        checks.neutral();
        checks.mode(RobotMode.TELEOPERATED, true, red);
        checks.container.getDrive().setPose(new Pose2d(8, 4, Rotation2d.ZERO));
        checks.joystick.setAButton(true);
        checks.joystick.setXButton(true);
        checks.joystick.setLeftTriggerAxis(1);
        checks.joystick.setRightBumperButton(true);
        checks.joystick.setLeftY(-0.6);
        checks.joystick.setRightX(-0.4);
        Pose2d start = checks.container.getDrive().getPose();
        checks.ticks(1);
        assertTrue(start.getTranslation().getDistance(checks.container.getDrive().getPose()
            .getTranslation()) > 0.5, "Shot controls must leave manual translation available");
        assertTrue(Math.abs(start.getRotation().minus(checks.container.getDrive().getRotation())
            .getRadians()) > 0.25, "Shot controls must leave manual rotation available");
        checks.joystick.setAButton(false);
        checks.joystick.setXButton(false);
        checks.joystick.setLeftTriggerAxis(0);
        checks.joystick.setRightBumperButton(false);
        checks.neutral();
        checks.mode(RobotMode.TELEOPERATED, false, red);
      }
      Files.createDirectories(Path.of("build/reports/simulation"));
      Files.write(Path.of("build/reports/simulation/drive-only-field-poses.csv"), checks.rows);
    }
  }

  public static void main(String[] args) throws Exception {
    run(true);
  }

  /** Exercise later coroutine phases, not just first-path tracking. */
  public static void runCoroutinePhases() throws Exception {
    try (var checks = new SimulationChecks(false)) {
      for (boolean red : new boolean[] {false, true}) {
        for (String name : List.of("Shoot Only", "Safe (Shoot First)",
            "Depot Double Pass", "Trench Depot Points", "Trench Outpost Disrupt",
            "Trench Depot Follow", "Depot Single Pass Shoot On Move")) {
          checks.neutral();
          checks.mode(RobotMode.AUTONOMOUS, false, red);
          var nt = NetworkTableInstance.getDefault();
          nt.getEntry("/Tunables/Autonomous/Chooser/selected/tune").setString(name);
          nt.flushLocal();
          nt.waitForListenerQueue(1);
          checks.ticks(0.1);
          assertEquals(name, checks.container.getSelectedAutoName());
          checks.mode(RobotMode.AUTONOMOUS, true, red);
          boolean fed = false;
          boolean droveAfterFeeding = false;
          boolean gathered = false;
          for (int cycle = 0; cycle < 2000; cycle++) { // Up to 40 s to reach later phases.
            checks.tick();
            fed |= checks.container.getIndexer().getGoal()
                == frc.robot.subsystems.indexer.Indexer.Goal.FEED;
            var velocity = checks.container.getDrive().getChassisVelocities();
            if (fed && checks.container.getIndexer().getGoal()
                == frc.robot.subsystems.indexer.Indexer.Goal.IDLE
                && Math.hypot(velocity.vx, velocity.vy) > 0.3) droveAfterFeeding = true;
            gathered |= Scheduler.getDefault().getRunningCommands().stream()
                .anyMatch(command -> command.name().equals("Gather Current Limit"));
          }
          assertTrue(fed, name + " must reach feeding on " + (red ? "red" : "blue")
              + "; goal=" + checks.container.getShooter().getGoal()
              + "; target RPM=" + checks.container.getShooter().getCommandedRPM()
              + "; measured RPM=" + checks.container.getShooter().getAverageVelocityRPM()
              + "; running=" + Scheduler.getDefault().getRunningCommands().stream()
                  .map(Command -> Command.name()).toList());
          if (name.contains("Trench") || name.equals("Depot Double Pass")) {
            assertTrue(droveAfterFeeding, name + " must leave shooting and resume driving");
          }
          if (name.equals("Trench Depot Follow")) {
            assertTrue(gathered, "Follow must reach its scoped gather-current monitor");
          }
          checks.mode(RobotMode.AUTONOMOUS, false, red);
          checks.ticks(0.2);
          assertEquals(frc.robot.subsystems.shooter.Shooter.Goal.IDLE,
              checks.container.getShooter().getGoal(), name + " shooter cancellation");
          assertEquals(frc.robot.subsystems.indexer.Indexer.Goal.IDLE,
              checks.container.getIndexer().getGoal(), name + " indexer cancellation");
          assertEquals(frc.robot.subsystems.intake.Intake.Goal.IDLE,
              checks.container.getIntake().getGoal(), name + " intake cancellation");
          assertTrue(Scheduler.getDefault().getRunningCommands().stream()
              .noneMatch(command -> command.name().contains("Aim With Vision Creep")
                  || command.name().contains("Gather Current Limit")
                  || command.name().contains("Moving Shot Drive")
                  || command.name().contains("Intake Periodic Auto Rehome")),
              name + " must cancel phase children");
          System.out.println("PASS coroutine phases " + name + " " + (red ? "red" : "blue"));
        }
      }
      // A long preload delay overlaps the shot instead of starting after it.
      checks.mode(RobotMode.AUTONOMOUS, false, false);
      var nt = NetworkTableInstance.getDefault();
      nt.getEntry("/Tunables/Autonomous/ShootFirstDelaySecs/tune").setDouble(6.0);
      nt.getEntry("/Tunables/Autonomous/Chooser/selected/tune").setString("Safe (Shoot First)");
      nt.flushLocal();
      nt.waitForListenerQueue(1);
      checks.ticks(0.1);
      checks.mode(RobotMode.AUTONOMOUS, true, false);
      Pose2d start = checks.container.getDrive().getPose();
      double started = Timer.getTimestamp();
      double firstMotion = Double.NaN;
      for (int cycle = 0; cycle < 400; cycle++) {
        checks.tick();
        if (start.getTranslation().getDistance(checks.container.getDrive().getPose().getTranslation()) > 0.03) {
          firstMotion = Timer.getTimestamp() - started;
          break;
        }
      }
      assertTrue(firstMotion >= 6 && firstMotion < 6.4,
          "Preload delay must overlap shooting; first motion at " + firstMotion);
      checks.mode(RobotMode.AUTONOMOUS, false, false);
      nt.getEntry("/Tunables/Autonomous/ShootFirstDelaySecs/tune").setDouble(0.0);
    }
  }

  @Override public void close() {
    neutral();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    Scheduler.getDefault().cancelAll();
    robot.close();
    fieldPoses.close();
    Logger.end();
    WPIUtilJNI.disableMockTime();
    SimHooks.resumeTiming();
    if (!playback) {
      NetworkTableInstance.getDefault().stopServer();
      savedPreferences.forEach((path, bytes) -> {
        try {
          if (bytes == null) Files.deleteIfExists(path);
          else Files.write(path, bytes);
        } catch (IOException e) { throw new RuntimeException(e); }
      });
    }
  }
}
