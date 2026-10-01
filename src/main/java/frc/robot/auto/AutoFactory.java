package frc.robot.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import frc.robot.util.V3Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;

/** Choreo trajectory loading and alliance flipping with Commands v3 execution. */
public final class AutoFactory {
  final Consumer<Pose2d> resetPose;
  final Consumer<SwerveSample> follower;
  final Mechanism drive;
  private final boolean allianceFlipping;
  private final Map<String, Trajectory<SwerveSample>> cache = new HashMap<>();

  public AutoFactory(Consumer<Pose2d> resetPose, Consumer<SwerveSample> follower,
      boolean allianceFlipping, Mechanism drive) {
    this.resetPose = resetPose;
    this.follower = follower;
    this.allianceFlipping = allianceFlipping;
    this.drive = drive;
  }

  boolean flip() {
    if (!allianceFlipping) return false;
    return MatchState.getAlliance().orElseThrow(
        () -> new IllegalStateException("Alliance must be known before starting autonomous")) == Alliance.RED;
  }

  Trajectory<SwerveSample> load(String name) {
    return cache.computeIfAbsent(name, key -> Choreo.<SwerveSample>loadTrajectory(key)
        .filter(trajectory -> !trajectory.samples().isEmpty())
        .orElseThrow(() -> new IllegalStateException("Missing or empty Choreo trajectory: " + key)));
  }

  public AutoRoutine newRoutine(String name) { return new AutoRoutine(this, name); }

  public Command resetOdometry(Supplier<Optional<Pose2d>> pose) {
    return V3Commands.runOnce(() -> {
      Pose2d initial = pose.get().orElseThrow(() -> new IllegalStateException("No initial auto pose"));
      resetPose.accept(flip() ? ChoreoAllianceFlipUtil.flip(initial) : initial);
    }, drive);
  }
}
