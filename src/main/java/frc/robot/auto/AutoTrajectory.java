package frc.robot.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Timer;

/** Samples Choreo paths and forks marker commands inside the trajectory's v3 coroutine scope. */
public final class AutoTrajectory {
  private record Binding(double time, Command command) {}
  private final AutoFactory factory;
  private final Trajectory<SwerveSample> trajectory;
  private final List<Binding> bindings = new ArrayList<>();

  AutoTrajectory(AutoFactory factory, Trajectory<SwerveSample> trajectory) {
    this.factory = factory;
    this.trajectory = trajectory;
  }
  public AutoTrajectory mirrorY() { return new AutoTrajectory(factory, trajectory.mirrorY()); }
  public Optional<Pose2d> getInitialPose() { return trajectory.getInitialPose(factory.flip()); }
  public Optional<Pose2d> getFinalPose() { return trajectory.getFinalPose(factory.flip()); }
  public Command resetOdometry() {
    return factory.drive.run(co -> factory.resetPose.accept(getInitialPose().orElseThrow()))
        .named("Reset " + trajectory.name());
  }
  public Marker atTime(String name) { return new Marker(name); }
  public final class Marker {
    private final String name;
    Marker(String name) { this.name = name; }
    public void onTrue(Command command) {
      var events = trajectory.getEvents(name);
      if (events.isEmpty()) throw new IllegalArgumentException("Missing marker " + name + " in " + trajectory.name());
      events.forEach(event -> bindings.add(new Binding(event.timestamp, command)));
    }
  }
  public Command cmd() {
    return factory.drive.run(co -> {
      boolean flip = factory.flip();
      var fired = new boolean[bindings.size()];
      Timer timer = Timer.createStarted();
      double duration = trajectory.getTotalTime();
      while (true) {
        double time = Math.min(timer.get(), duration);
        factory.follower.accept(trajectory.sampleAt(time, flip).orElseThrow());
        for (int i = 0; i < bindings.size(); i++) {
          Binding binding = bindings.get(i);
          if (!fired[i] && time >= binding.time()) {
            fired[i] = true;
            co.fork(binding.command());
          }
        }
        // Give final-time markers a scheduler cycle before the scope closes.
        co.yield();
        if (time >= duration) break;
      }
    }).named("Follow " + trajectory.name());
  }
}
