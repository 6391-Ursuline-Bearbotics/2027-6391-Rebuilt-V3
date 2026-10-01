package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.util.V3Commands;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.system.RobotController;

class CommandsV3MigrationTest {
  private Scheduler scheduler;
  private final AtomicLong clock = new AtomicLong(1_000_000_000);
  private Mechanism drive;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize());
    RobotController.setTimeSource(clock::get);
    scheduler = Scheduler.createIndependentScheduler();
    drive = new Mechanism() {
      @Override public Scheduler getRegisteredScheduler() { return scheduler; }
    };
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    RobotController.setTimeSource(RobotController::getMonotonicTime);
  }

  private void tick() {
    scheduler.run();
    clock.addAndGet(20_000_000);
  }

  @Test
  void decoratorsCleanUpOnceOnCompletionAndCancellation() {
    var starts = new AtomicInteger();
    var ends = new ArrayList<Boolean>();
    Command completed = V3Commands.finallyDo(
        V3Commands.beforeStarting(V3Commands.none(), starts::incrementAndGet), ends::add);
    scheduler.schedule(completed);
    tick();
    assertEquals(List.of(false), ends);
    scheduler.cancel(completed);
    assertEquals(List.of(false), ends);

    Command running = V3Commands.finallyDo(
        V3Commands.beforeStarting(V3Commands.run(() -> {}, drive), starts::incrementAndGet),
        ends::add);
    scheduler.schedule(running);
    tick();
    scheduler.cancel(running);
    scheduler.cancel(running);
    assertEquals(List.of(false, true), ends);
    scheduler.schedule(running);
    tick();
    scheduler.cancel(running);
    assertEquals(3, starts.get());
    assertEquals(List.of(false, true, true), ends);
  }

  @Test
  void deadlineCancelsCompanionAndReleasesDrive() {
    var stopped = new AtomicInteger();
    Command group = V3Commands.deadline(V3Commands.waitSeconds(0.04),
        V3Commands.startEnd(() -> {}, stopped::incrementAndGet, drive));
    scheduler.schedule(group);
    for (int i = 0; i < 6; i++) tick();
    assertFalse(scheduler.isScheduledOrRunning(group));
    assertEquals(1, stopped.get());
    var replacementRuns = new AtomicInteger();
    scheduler.schedule(V3Commands.runOnce(replacementRuns::incrementAndGet, drive));
    tick();
    assertEquals(1, replacementRuns.get());
  }

  private AutoTrajectory trajectory(List<SwerveSample> samples) {
    var factory = new AutoFactory(pose -> {}, samples::add, false, drive);
    return new AutoTrajectory(factory, new Trajectory<>("Test",
        List.of(sample(0), sample(0.10)), List.of(0),
        List.of(new EventMarker(0.04, "Intake"), new EventMarker(0.10, "Finish"))));
  }

  private static SwerveSample sample(double time) {
    return new SwerveSample(time, time, 2, 0, 1, 0, 0, 0, 0, 0,
        new double[4], new double[4]);
  }

  @Test
  void trajectoryFiresMarkersOnceIncludingFinalTimestampAndCanRestart() {
    var samples = new ArrayList<SwerveSample>();
    var path = trajectory(samples);
    var intake = new AtomicInteger();
    var finish = new AtomicInteger();
    path.atTime("Intake").onTrue(V3Commands.runOnce(intake::incrementAndGet));
    path.atTime("Finish").onTrue(V3Commands.runOnce(finish::incrementAndGet));
    Command command = path.cmd();
    for (int run = 1; run <= 2; run++) {
      scheduler.schedule(command);
      for (int i = 0; i < 10; i++) tick();
      assertFalse(scheduler.isScheduledOrRunning(command));
      assertEquals(run, intake.get());
      assertEquals(run, finish.get());
      assertEquals(0.10, samples.getLast().t, 1e-9);
    }
  }

  @Test
  void cancelingRoutineStopsSamplingAndCancelsMarkerChildren() {
    var samples = new ArrayList<SwerveSample>();
    var path = trajectory(samples);
    var started = new AtomicInteger();
    var stopped = new AtomicInteger();
    path.atTime("Intake").onTrue(
        V3Commands.startEnd(started::incrementAndGet, stopped::incrementAndGet));
    var factory = new AutoFactory(pose -> {}, sample -> {}, false, drive);
    var routine = factory.newRoutine("Cancel test");
    routine.active().onTrue(path.cmd());
    Command command = routine.cmd();
    for (int i = 0; i < 4; i++) {
      if (i == 0) scheduler.schedule(command);
      tick();
    }
    assertEquals(1, started.get());
    scheduler.cancel(command);
    int sampleCount = samples.size();
    for (int i = 0; i < 4; i++) tick();
    assertEquals(sampleCount, samples.size());
    assertEquals(1, stopped.get());
    assertFalse(scheduler.isScheduledOrRunning(command));
  }

  @Test
  void deployedTrajectoriesLoadWithCurrentChoreoSchema() throws Exception {
    try (var paths = Files.list(Path.of("src/main/deploy/choreo"))) {
      var trajectories = paths.filter(path -> path.toString().endsWith(".traj")).toList();
      assertFalse(trajectories.isEmpty());
      for (Path path : trajectories) {
        String name = path.getFileName().toString().replace(".traj", "");
        var loaded = Choreo.<SwerveSample>loadTrajectory(name);
        assertTrue(loaded.isPresent(), name);
        assertFalse(loaded.orElseThrow().samples().isEmpty(), name);
      }
    }
  }
}
