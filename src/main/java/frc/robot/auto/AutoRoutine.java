package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import java.util.function.Consumer;

/** Routine lifetime owns all commands it forks; cancellation cancels the entire routine. */
public final class AutoRoutine {
  private final AutoFactory factory;
  private final String name;
  private final List<Command> starts = new ArrayList<>();
  private Runnable cancellationCleanup = () -> {};

  AutoRoutine(AutoFactory factory, String name) { this.factory = factory; this.name = name; }
  public AutoTrajectory trajectory(String path) { return new AutoTrajectory(factory, factory.load(path)); }
  public Activation active() { return new Activation(); }
  /** Declare a routine as ordinary sequential code with scoped coroutine children. */
  public void run(Consumer<Coroutine> body) {
    starts.add(Command.noRequirements(body).named(name + " Flow"));
  }
  public void onCancel(Runnable cleanup) { cancellationCleanup = cleanup; }
  public final class Activation {
    public void onTrue(Command command) { starts.add(command); }
  }
  public Command cmd() {
    return Command.noRequirements(co -> {
      co.awaitAll(starts);
      co.park();
    }).whenCanceled(cancellationCleanup).named(name);
  }
}
