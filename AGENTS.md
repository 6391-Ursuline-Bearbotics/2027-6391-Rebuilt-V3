# FRC Team 6391 — Rebuilt V3

## Scope and source of truth

These instructions apply to this repository, a WPILib 2027 Alpha 7 SystemCore robot project intended to demonstrate Commands v3. Preserve Commands v3 throughout robot behavior and autonomous execution.

Verify migration assumptions against `build.gradle`, `settings.gradle`, `.wpilib/wpilib_preferences.json`, `vendordeps/`, and the installed library sources. Historical Alpha 5/6 notes contained outdated dependency and API assumptions.

## Toolchain and build

- WPILib/GradleRIO: `2027.0.0-alpha-7`.
- Local Windows installation: `C:\Users\Public\wpilib\2027_alpha7`.
- Java: 25, supplied in that installation's `jdk` directory. The system Java may be older.
- Gradle: use the checked-in wrapper (currently 9.4.1).
- Entry point: `first.Main`, which starts `frc.robot.Robot`.
- Target: SystemCore, team 6391. Static assets deploy to `/home/systemcore/deploy`.

Build from the repository root in PowerShell:

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2027_alpha7\jdk'
.\gradlew.bat build --console=plain
```

Use the installed Alpha 7 Java Gradle template to verify packaging and deployment APIs. It uses the `application` plugin, `deployArtifact.configureApplication(application)`, and `wpi.java.configureApplication(application)`. Simulation uses `wpi.java.runSimWithDebugJni`; deployment JNI debugging is configured on the deployment artifact.

Prefer the Alpha 7 WPILib VS Code installation. Alpha 5's simulator requests `simulateExternalJavaRelease` and reads `build/sim/release_java.json`; the compatibility task in `build.gradle` delegates to Alpha 7's `simulateExternalJava` and copies its `java.json` metadata to that filename. Alpha 7's actual application launch task is `run`.

Report compilation, tests, simulation startup, and robot deployment separately. A successful build does not prove hardware operation. Do not deploy unless requested.

## Dependencies and migration

Current manifests declare Phoenix 6 `26.70.0-alpha-2`, REVLib `2027.0.0-alpha-8`, AdvantageKit `27.0.0-alpha-6`, Choreo `2027.0.0-alpha-3`, and PhotonLib `dev-v2027.0.0-alpha-2-73-g4651dbae`. Their alpha numbers need not match WPILib. Check actual artifacts and compatibility rather than trusting the filename or `wpilibYear`.

- Commands v3's official Alpha 7 artifact is `org.wpilib:commandsv3-java`. Obtain the manifest from the installed WPILib vendordeps directory.
- Choreo's library-provided AutoFactory, routines, commands, and triggers use Commands v2. This project uses only its trajectory loader, samples, and transformations, with local Commands v3 adapters in `frc.robot.auto`. Do not add Commands v2 to bypass the conflict check.
- AdvantageKit `@AutoLog` inputs require annotation processing. Generated `*InputsAutoLogged` files belong in build output, not handwritten source.
- REAL uses Pigeon2 and Limelight. PhotonVision simulation is enabled with the pinned dev manifest from the GitHub Dev release; its matching source explicitly targets WPILib Alpha 7 and its JNI supports SystemCore. Do not relabel 2026 manifests as 2027.
- SIM uses PhotonVision with the explicit 2026 REBUILT tag layout, existing rear camera transform, and an ideal 50 FPS, zero-noise, zero-latency camera. Camera motion comes from independent wheel odometry (`Drive.getSimulationPose`), never the vision-corrected pose. Single-tag solutions use a pose reference to disambiguate. The dev multi-tag solver produced an invalid planar solve during regression testing; simulated solves more than 0.5 in the translation-plus-heading reference metric are dropped and counted at `/Telemetry/Vision/Simulation/RejectedSolves`. This is a simulation-only workaround using known simulated motion, not a real-camera accuracy guarantee. Revisit it when upgrading PhotonLib or adding latency/noise.
- Hood servos connect directly to SystemCore SmartIO PWM channels 0 and 1. `ShooterHoodIOServo` uses Alpha 7's `PWM` API, a 20 ms period, and the existing normalized angle mapping. Pulse endpoints are configurable in `ShooterConstants` (600–2400 microseconds); verify calibration with the installed servos.

The Alpha 7 regression suite has thirteen automated tests. Tests cover coroutine cleanup/restart and bounded waits, deadline cancellation, trajectory markers, deployed path loading, simulation startup, SmartIO servo pulse mapping, simulator functionality, later autonomous phases on both alliances, shooter simulation requests/gain units, and actual PhotonVision tag observations with pose and timestamp validation. Hardware operation and deployment have not been verified.

Verified Alpha 7 API changes:

- Commands v3 `Mechanism` is an interface; implementations use `implements Mechanism` and register periodic callbacks with `Scheduler`. `Trigger` is in `org.wpilib.command3`.
- Match information belongs to `MatchState`, enabled/mode checks to `RobotState`, and alliance values are `Alliance.RED` and `Alliance.BLUE`.
- Math uses `ChassisVelocities`, `SwerveModuleVelocity`, `Models`, and geometry `ZERO` constants. Swerve optimization, cosine scaling, and wheel velocity desaturation return new values; always use their results.
- Field layouts use `org.wpilib.fields.Field` and `Fields`. Preserve the 2026 REBUILT field used by this robot; the toolchain year does not establish the game field.
- Alerts require a unique identifier, text, and `Alert.Level`. REV reset/persist modes are in `com.revrobotics`.
- Use Alpha 7 `Tunables` for configuration and `Telemetry` for outputs. Do not use the legacy dashboard namespace. The chooser is `/Tunables/Autonomous/Chooser` (write `selected/tune`, read `selected/value`); the shoot-first delay is `/Tunables/Autonomous/ShootFirstDelaySecs`. Field2D outputs are `/Telemetry/Drive/Field` and `/Telemetry/Autonomous/Preview`. WPILib's Field2d type still lives in its historical `org.wpilib.smartdashboard` package.
- Timestamp APIs are `Timer.getTimestamp()` (seconds) and `RobotController.getTime()` (nanoseconds). NetworkTables timestamps and last-change values are also nanoseconds in Alpha 7. Convert explicitly when calculating observation times or freshness checks; camera-provided latency remains milliseconds.

## Architecture

- `Robot`: AdvantageKit logging and robot lifecycle.
- `RobotContainer`: REAL/SIM/REPLAY IO selection, controller bindings, autonomous chooser and preview.
- `AutoRoutines`: direct Commands v3 coroutine flows (`AutoRoutine.run`), with sequential path awaits, local per-run state, bounded aim waits, and scoped aiming/rehome/current-monitoring children. Preserve existing readiness policies: trench aiming times out after 2 s and then feeds; moving-shot feeding waits indefinitely for shooter readiness. Cancellation idles all auto mechanism goals, stops drive, and clears speed caps.
- `auto`: Choreo loading, alliance flipping, path sampling, scoped event-marker commands, and dashboard selection implemented with Commands v3. Marker children cancel with their trajectory; routine children cancel with their routine. An unknown alliance prevents starting an alliance-flipped path.
- `subsystems/drive`: swerve modules, Pigeon2 gyro, odometry and pose estimation. CTRE configuration lives in `generated/TunerConstants.java`.
- `subsystems/intake`: TalonFX deploy arm and roller, with jam recovery.
- `subsystems/indexer`: TalonFX belt/kicker and REV SparkMax spinners.
- `subsystems/shooter`: TalonFX flywheel, hood IO and distance lookup tables.
- `subsystems/vision`: Limelight, PhotonVision simulation and QuestNav IO feeding drive pose estimation.
- `util`: Phoenix configuration helpers, logged tunables, and `V3Commands` coroutine recipes. Cancellation cleanup belongs in `Command.onCancel`/`whenCanceled`; an abandoned coroutine does not reliably unwind Java `finally` blocks.
- `src/main/deploy`: runtime assets and autonomous paths.

Preserve the IO pattern: hardware access belongs in hardware IO, physics in simulation IO, and robot behavior in subsystem/container/command code. Preserve all three runtime modes.

## Working conventions

- Inspect the working tree and preserve existing user edits.
- Preserve CAN IDs, inversions, gearing, gains, current limits, field conventions, and autonomous timing unless the task calls for changes.
- Check API adaptations against the exact Alpha 7 sources/JARs; do not copy obsolete command examples from historical notes.
- Register mechanism periodic callbacks with the Commands v3 scheduler exactly once. Keep the scheduler and all coroutine command operations on the same thread.
- Use direct coroutine bodies for autonomous phase orchestration; keep timers, debouncers, controllers, and latches local to each execution. Use `co.fork` for phase children and let scope exit cancel them. Explicitly cancel rehome work before changing the intake to collection. Do not replace these flows with compatibility sequences/deadline groups.
- Run the wrapper build after code or dependency changes. Use focused tests for scheduling, cancellation, timing, or behavior changes when practical.
- Explicitly document temporarily excluded integrations and runtime limitations. Do not silently replace unavailable hardware with successful-looking no-op IO.
- Update these instructions when the migration is resolved or the toolchain changes. Keep historical speculation out of standing instructions.

## Simulator regression checks

With the Alpha 7 JDK selected, run `./gradlew.bat test --tests frc.robot.SimulationFunctionalTest --console=plain` for deterministic 20 ms simulation. `./gradlew.bat simulateChecks --console=plain` replays the same checks at real-time speed with the simulator GUI (a 15 s initial pause allows opening the field views). These tasks construct the actual Robot and physics IO, feed HAL driver-station/controller data, use AdvantageKit's before/after hooks, and run the normal robot lifecycle and Commands v3 scheduler on one thread. Test classes run in separate JVMs to isolate global HAL/logger/scheduler state.

Scenarios cover forward, strafe, rotation, joystick release, disabled joystick input, and Depot Cycle, Depot Inside, Safe, and mirrored Trench Depot Points on blue and red. Path checks assert position and heading, autonomous chooser tuning, preview mirroring, every published Field2D pose, cancellation on disable, and auto-to-teleop transitions. Safe includes its bounded shooting phase; Trench Depot Points checks its first drive phase and cancellation, not the complete later shooting/gather sequence. Tests synchronize the NetworkTables mock clock with stepped HAL time so PhotonVision frames follow simulation time. Simulation supplies ideal camera observations but does not model obstacles, bumps, collected game pieces, or physical shooter accuracy.

`./gradlew.bat test --tests frc.robot.AutonomousCoroutineTest --console=plain` additionally exercises feeding and shooting-to-driving transitions in seven routine families on both alliances, the Follow gather-current scope, child/goal cleanup on disable, and overlapping the preload shot with a 6 s minimum delay. These checks validate later phase progression, not exact later trajectory tracking. Shooter simulation converts Phoenix gains from rotations/sec to radians/sec and approximates FOC velocity requests with its voltage-based motor model; hardware torque-current limits and bang-bang transients are not modeled.

Path tolerances are 35 cm peak/15 cm endpoint position and 20° peak/5° endpoint heading. Trench Depot Points has the largest measured transient error (about 27 cm/19°), while all checked endpoints are within 7 cm. The CSV trace is written to `build/reports/simulation/field-poses.csv`. Run `python scripts/plot_simulation.py` to render those recorded Field2D poses against their expected paths.
