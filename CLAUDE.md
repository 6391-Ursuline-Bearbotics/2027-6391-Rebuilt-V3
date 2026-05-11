# FRC Team 6391 — 2027 Robot (Rebuilt V3)

## Project Overview

FRC team 6391's 2027 robot code, migrated from the WPILib 2026 codebase to **WPILib 2027 alpha 6** running on **SystemCore** hardware.

## WPILib Version

- **Installed folder:** `C:\Users\Public\wpilib\2027_alpha5\` (folder label is alpha5 but alpha 6 is installed)
- **Gradle plugin:** `2027.0.0-alpha-6` (in `build.gradle`)
- **settings.gradle:** `wpilibYear = '2027_alpha5'` — matches the actual installed folder name
- **Target:** SystemCore (not RoboRIO)
- **Java:** 25

## Commands V3 Migration

This project uses **Commands V3** (`org.wpilib.command3`). Key differences from V2:

### Package renames
- `org.wpilib.command2.*` → `org.wpilib.command3.*`
- `SubsystemBase` → `Mechanism`
- `CommandScheduler.getInstance()` → `Scheduler.getDefault()`
- `CommandXboxController` → `CommandNiDsXboxController`

### Mechanism (formerly SubsystemBase)
- No `@Override periodic()` — `Mechanism` has no `periodic()` hook
- Register periodic logic in the constructor: `Scheduler.getDefault().addPeriodic(this::periodic)`

### Command creation (V3 patterns)
```java
// One-shot (replaces Commands.runOnce)
Command.noRequirements(co -> action()).named("Name")

// Continuous with subsystem requirement (replaces Commands.run(body, subsystem))
subsystem.runRepeatedly(() -> action()).named("Name")

// Coroutine with init + loop (replaces Commands.run + beforeStarting)
subsystem.run(co -> {
    init();
    while (true) {
        action();
        co.yield();
    }
}).named("Name")

// Wait for trigger/condition inside a coroutine
while (!condition()) { co.yield(); }

// Park (suspend until cancelled — replaces Commands.run for hold-while-pressed)
co.park();

// Composition
Command.sequence(a, b, c).withAutomaticName()
Command.parallel(a, b).withAutomaticName()
Command.waitFor(Seconds.of(2.0)).withAutomaticName()
Command.waitUntil(() -> condition).withAutomaticName()

// Cleanup on cancel (replaces finallyDo for always-cancelled commands)
command.whenCanceled(() -> cleanup())

// Timeout (argument type changed)
command.withTimeout(Seconds.of(2.0))

// Composition on built Command (returns builder, needs .withAutomaticName())
builtCommand.until(condition).withAutomaticName()
builtCommand.alongWith(other).withAutomaticName()
builtCommand.andThen(other).withAutomaticName()
```

### Removed in V3
- `Commands` utility class — use `Command.*` static methods
- `SysIdRoutine` from `command3` — use `org.wpilib.sysid` (see TODO in Drive.java)
- `.ignoringDisable(true)` — no equivalent in V3
- `.beforeStarting(body)` — inline init at top of coroutine body
- `Commands.defer(supplier, reqs)` — use coroutine with local variables instead

## Known Broken Dependencies

These vendordeps are **not yet available** for WPILib 2027 and will cause compile errors:

| Dependency | Status | Affected files |
|---|---|---|
| **CTRE Phoenix 6** | Not available for 2027 | `TunerConstants`, `GyroIOPigeon2`, `ModuleIOTalonFX`, `ModuleIOSim`, `PhoenixOdometryThread`, all CTRE hardware IOs |
| **AdvantageKit (AK)** | Not available for 2027 | `Robot.java` (LoggedRobot), all `@AutoLogOutput`, `*InputsAutoLogged`, `Logger.*` |
| **Choreo** | Not available for 2027 | `AutoRoutines.java`, auto preview in `RobotContainer.java` |

`AutoRoutines.java` also still needs its `Commands.*` calls migrated to V3 once Choreo is available.

## Subsystems

- **Drive** — Swerve drive (4x TalonFX drive + TalonFX turn + CANcoder). Broken until CTRE.
- **Intake** — Deploy arm (TalonFX) + roller (TalonFX). Jam detection with auto-reverse.
- **Indexer** — Belt (TalonFX) + kicker (TalonFX) + left/right spinners (SparkMax). Jam detection + spinner inrush current management.
- **Shooter** — Flywheel (TalonFX) + servo hood. Distance-based LUT for RPM/hood angle. Broken until CTRE.
- **Vision** — Limelight (AprilTag + MegaTag2) + PhotonVision sim. Supports QuestNav. Feeds drive pose estimator.

## Controllers

- **Driver (`drv`):** `CommandNiDsXboxController(0)`
- **Operator (`op`):** `CommandNiDsXboxController(1)`

Note: `.getHID().setRumble()` calls may need updating for the NiDs input system API.
