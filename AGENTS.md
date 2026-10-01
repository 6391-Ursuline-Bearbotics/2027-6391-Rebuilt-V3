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

Report compilation, tests, simulation startup, and robot deployment separately. A successful build does not prove hardware operation. Do not deploy unless requested.

## Dependencies and migration

Current manifests declare Phoenix 6 `26.70.0-alpha-2`, REVLib `2027.0.0-alpha-8`, AdvantageKit `27.0.0-alpha-6`, and Choreo `2027.0.0-alpha-3`. Their alpha numbers need not match WPILib. Check actual artifacts and compatibility rather than trusting the filename or `wpilibYear`.

- Commands v3's official Alpha 7 artifact is `org.wpilib:commandsv3-java`. Obtain the manifest from the installed WPILib vendordeps directory.
- Choreo's library-provided AutoFactory, routines, commands, and triggers use Commands v2. This project uses only its trajectory loader, samples, and transformations, with local Commands v3 adapters in `frc.robot.auto`. Do not add Commands v2 to bypass the conflict check.
- AdvantageKit `@AutoLog` inputs require annotation processing. Generated `*InputsAutoLogged` files belong in build output, not handwritten source.
- `GyroIONavX.java` and `VisionIOPhotonVisionSim.java` are preserved but excluded in `build.gradle` because compatible vendordeps are not installed. REAL uses Pigeon2 and Limelight. SIM explicitly alerts that camera simulation is disabled and supplies no vision observations. Verify compatible releases before enabling these integrations; do not relabel 2026 manifests as 2027.
- Hood servos connect directly to SystemCore SmartIO PWM channels 0 and 1. `ShooterHoodIOServo` uses Alpha 7's `PWM` API, a 20 ms period, and the existing normalized angle mapping. Pulse endpoints are configurable in `ShooterConstants` (600–2400 microseconds); verify calibration with the installed servos.

The Alpha 7 build and seven automated migration tests pass. Tests cover coroutine cleanup, deadline cancellation, trajectory markers, deployed path loading, disabled simulation startup, and SmartIO servo pulse mapping in HAL simulation. Hardware operation and deployment have not been verified.

Verified Alpha 7 API changes:

- Commands v3 `Mechanism` is an interface; implementations use `implements Mechanism` and register periodic callbacks with `Scheduler`. `Trigger` is in `org.wpilib.command3`.
- Match information belongs to `MatchState`, enabled/mode checks to `RobotState`, and alliance values are `Alliance.RED` and `Alliance.BLUE`.
- Math uses `ChassisVelocities`, `SwerveModuleVelocity`, `Models`, and geometry `ZERO` constants. Swerve optimization, cosine scaling, and wheel velocity desaturation return new values; always use their results.
- Field layouts use `org.wpilib.fields.Field` and `Fields`. Preserve the 2026 REBUILT field used by this robot; the toolchain year does not establish the game field.
- Alerts require a unique identifier, text, and `Alert.Level`. REV reset/persist modes are in `com.revrobotics`.
- Dashboard objects use `Tunables` rather than removed `SmartDashboard` methods. Preserve established NetworkTables paths when adapting dashboard input/output.
- Timestamp APIs are `Timer.getTimestamp()` (seconds) and `RobotController.getTime()` (nanoseconds). NetworkTables timestamps and last-change values are also nanoseconds in Alpha 7. Convert explicitly when calculating observation times or freshness checks; camera-provided latency remains milliseconds.

## Architecture

- `Robot`: AdvantageKit logging and robot lifecycle.
- `RobotContainer`: REAL/SIM/REPLAY IO selection, controller bindings, autonomous chooser and preview.
- `AutoRoutines`: Choreo trajectories and autonomous sequences.
- `auto`: Choreo loading, alliance flipping, path sampling, scoped event-marker commands, and dashboard selection implemented with Commands v3. Marker children cancel with their trajectory; routine children cancel with their routine. An unknown alliance prevents starting an alliance-flipped path.
- `subsystems/drive`: swerve modules, Pigeon2 gyro, odometry and pose estimation. CTRE configuration lives in `generated/TunerConstants.java`.
- `subsystems/intake`: TalonFX deploy arm and roller, with jam recovery.
- `subsystems/indexer`: TalonFX belt/kicker and REV SparkMax spinners.
- `subsystems/shooter`: TalonFX flywheel, hood IO and distance lookup tables.
- `subsystems/vision`: Limelight, optional PhotonVision simulation and QuestNav IO feeding drive pose estimation.
- `util`: Phoenix configuration helpers, logged tunables, and `V3Commands` coroutine recipes. Cancellation cleanup belongs in `Command.onCancel`/`whenCanceled`; an abandoned coroutine does not reliably unwind Java `finally` blocks.
- `src/main/deploy`: runtime assets and autonomous paths.

Preserve the IO pattern: hardware access belongs in hardware IO, physics in simulation IO, and robot behavior in subsystem/container/command code. Preserve all three runtime modes.

## Working conventions

- Inspect the working tree and preserve existing user edits.
- Preserve CAN IDs, inversions, gearing, gains, current limits, field conventions, and autonomous timing unless the task calls for changes.
- Check API adaptations against the exact Alpha 7 sources/JARs; do not copy obsolete command examples from historical notes.
- Register mechanism periodic callbacks with the Commands v3 scheduler exactly once. Keep the scheduler and all coroutine command operations on the same thread.
- Run the wrapper build after code or dependency changes. Use focused tests for scheduling, cancellation, timing, or behavior changes when practical.
- Explicitly document temporarily excluded integrations and runtime limitations. Do not silently replace unavailable hardware with successful-looking no-op IO.
- Update these instructions when the migration is resolved or the toolchain changes. Keep historical speculation out of standing instructions.
