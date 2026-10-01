package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Command;

/** Routine lifetime owns all commands it forks; cancellation cancels the entire routine. */
public final class AutoRoutine {
  private final AutoFactory factory;
  private final String name;
  private final List<Command> starts = new ArrayList<>();

  AutoRoutine(AutoFactory factory, String name) { this.factory = factory; this.name = name; }
  public AutoTrajectory trajectory(String path) { return new AutoTrajectory(factory, factory.load(path)); }
  public Activation active() { return new Activation(); }
  public final class Activation {
    public void onTrue(Command command) { starts.add(command); }
  }
  public Command cmd() {
    return Command.noRequirements(co -> {
      co.awaitAll(starts);
      co.park();
    }).named(name);
  }
}
