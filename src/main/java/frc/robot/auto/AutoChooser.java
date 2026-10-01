package frc.robot.auto;

import frc.robot.util.V3Commands;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableTable;

/** Dashboard selectable that constructs Commands v3 routines when autonomous starts. */
public final class AutoChooser implements ComplexTunable {
  private record Choice(String name, Supplier<Command> command) {}
  private final Selectable<Choice> selectable = new Selectable<>();
  public AutoChooser() { selectable.addDefault("Nothing", new Choice("Nothing", V3Commands::none)); }
  public void addRoutine(String name, Supplier<AutoRoutine> routine) {
    addCmd(name, () -> routine.get().cmd());
  }
  public void addCmd(String name, Supplier<Command> command) {
    selectable.add(name, new Choice(name, command));
  }
  public Command selectedCommand() { return selectable.getSelected().command().get(); }
  public String selectedName() { return selectable.getSelected().name(); }
  @Override public String getTunableType() { return selectable.getTunableType(); }
  @Override public void publishTunable(TunableTable table) { selectable.publishTunable(table); }
}
