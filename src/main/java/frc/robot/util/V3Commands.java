package frc.robot.util;

import static org.wpilib.units.Units.Seconds;

import java.util.Arrays;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.ParallelGroupBuilder;

/** Reusable coroutine recipes. All returned commands run under the Commands v3 scheduler. */
public final class V3Commands {
  private V3Commands() {}

  public static Command runOnce(Runnable body, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements)).executing(co -> body.run()).named("Run Once");
  }

  public static Command run(Runnable body, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements)).executing(co -> {
      while (true) {
        body.run();
        co.yield();
      }
    }).named("Run Repeatedly");
  }

  public static Command startEnd(Runnable start, Runnable end, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements)).executing(co -> {
      start.run();
      co.park();
    }).whenCanceled(end).named("Start End");
  }

  public static Command none() { return runOnce(() -> {}); }
  public static Command print(String message) { return runOnce(() -> System.out.println(message)); }
  public static Command waitSeconds(double seconds) {
    return Command.waitFor(Seconds.of(seconds)).named("Wait " + seconds + "s");
  }
  public static Command waitUntil(BooleanSupplier condition) {
    return Command.waitUntil(condition).named("Wait Until");
  }
  public static Command sequence(Command... commands) {
    return Command.sequence(commands).withAutomaticName();
  }
  public static Command parallel(Command... commands) {
    return Command.parallel(commands).withAutomaticName();
  }
  public static Command deadline(Command deadline, Command... companions) {
    return new ParallelGroupBuilder().requiring(deadline).optional(companions).withAutomaticName();
  }
  public static Command defer(Supplier<Command> supplier, Set<Mechanism> requirements) {
    return Command.requiring(requirements).executing(co -> co.await(supplier.get())).named("Deferred");
  }
  public static Command repeatedly(Command command) {
    return Command.requiring(command.requirements()).executing(co -> {
      while (true) {
        co.await(command);
        co.yield();
      }
    }).withPriority(command.priority()).named(command.name() + " Repeated");
  }

  public static Command beforeStarting(Command command, Runnable init) {
    return new Decorated(command, command.name(), init, null);
  }
  public static Command named(Command command, String name) {
    return new Decorated(command, name, null, null);
  }
  public static Command finallyDo(Command command, Runnable cleanup) {
    return finallyDo(command, interrupted -> cleanup.run());
  }
  public static Command finallyDo(Command command, Consumer<Boolean> cleanup) {
    return new Decorated(command, command.name(), null, cleanup);
  }

  // Delegate execution directly so requirement ownership, priority and cancellation stay intact.
  private record Decorated(Command command, String name, Runnable init, Consumer<Boolean> cleanup)
      implements Command {
    @Override public Set<Mechanism> requirements() { return command.requirements(); }
    @Override public int priority() { return command.priority(); }
    @Override public void run(Coroutine co) {
      if (init != null) init.run();
      command.run(co);
      if (cleanup != null) cleanup.accept(false);
    }
    @Override public void onCancel() {
      try { command.onCancel(); }
      finally { if (cleanup != null) cleanup.accept(true); }
    }
  }
}
