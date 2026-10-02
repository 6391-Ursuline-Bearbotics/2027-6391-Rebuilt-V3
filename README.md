# Team 6391 — REBUILT V3

Java robot code for Team 6391's 2026 REBUILT robot, running on **SystemCore with WPILib 2027 Alpha 7**. The project is intended to demonstrate **Commands v3**, with swerve drive, intake, indexer, flywheel shooter, and autonomous paths loaded from Choreo.

The toolchain is for 2027; the robot geometry, AprilTag layout, and autonomous assets use the **2026 REBUILT field**.

## Getting started

Use the WPILib Alpha 7 installation and its Java 25 JDK. This workspace uses:

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2027_alpha7\jdk'
.\gradlew.bat build --console=plain
```

Use the checked-in Gradle wrapper (9.4.1). An older system Java installation will not work. The application entry point is `first.Main`, which starts `frc.robot.Robot`.

To start the interactive simulator:

```powershell
.\gradlew.bat run --console=plain
```

Prefer the Alpha 7 WPILib VS Code installation for **Simulate Robot Code**. A compatibility task also supports Alpha 5 launchers that request `simulateExternalJavaRelease` and `build/sim/release_java.json`.

For hardware deployment, use the WPILib deploy action or, when connected to the robot:

```powershell
.\gradlew.bat deploy --console=plain
```

The deployment target is SystemCore, team 6391. Files from `src/main/deploy` deploy to `/home/systemcore/deploy`. Hardware operation and deployment have not been verified by the simulator tests.

## Commands v3: current state

The robot runs the Commands v3 scheduler and uses `org.wpilib.command3` throughout its command code. There is no Commands v2 vendordep. Autonomous orchestration now uses **direct coroutine flows**: ordinary sequential statements, local timers and controllers, path awaits, bounded condition waits, and scoped concurrent children. `AutoRoutines` no longer uses `V3Commands`, sequence builders, or parallel/deadline groups. Some subsystem convenience commands and characterization recipes still use `V3Commands`; those execute on the same v3 scheduler.

Commands v3 allows a command body to describe a process directly:

- Local variables and ordinary control flow persist across `co.yield()`. Yield returns control to the scheduler; execution resumes on a later scheduler cycle.
- `co.wait(...)` and `co.waitUntil(...)` suspend the command while other robot work continues. A timed condition wait returns a result that the routine can branch on.
- `co.await(child)` runs or joins a child and waits for completion. `co.awaitAll(...)` joins multiple children; `co.awaitAny(...)` finishes when one exits and cancels the rest.
- `co.fork(child)` starts concurrent work within the parent's lifetime. Children cannot outlive their parent; they cancel when that scope exits.
- `co.park()` holds the command open until cancellation. It does not represent successful completion.

These operations are cooperative scheduling on the scheduler thread, not permission to start background threads or block with `Thread.sleep()` in robot commands. A repeating control loop must yield. Mechanism requirements and priorities still govern access to shared hardware, including when children are forked.

Examples already in this repository:

| Source | Coroutine behavior |
| --- | --- |
| [`AutoRoutines.java`](src/main/java/frc/robot/AutoRoutines.java) | Declares each routine with `routine.run(co -> ...)`. A shooting scope forks aiming and rehome commands, waits for aim/readiness and feed duration, then returns to cancel its children before the next drive phase. |
| [`auto/AutoTrajectory.java`](src/main/java/frc/robot/auto/AutoTrajectory.java) | Keeps a timer and marker state in the command body, samples the path each cycle, forks marker commands, and yields. Marker commands cancel with the trajectory. |
| [`auto/AutoRoutine.java`](src/main/java/frc/robot/auto/AutoRoutine.java) | Awaits routine children and then parks until the routine is canceled. |
| [`subsystems/drive/Drive.java`](src/main/java/frc/robot/subsystems/drive/Drive.java) | SysId waits for settling, maintains a local timer, applies voltage in a yielding loop, and explicitly stops on completion or cancellation. |
| [`commands/DriveCommands.java`](src/main/java/frc/robot/commands/DriveCommands.java) | Uses yielding control loops for drive commands. |
| [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java) | Several controller actions use direct coroutine loops or park while a button remains held. |

For example, the existing SysId command has this structure, with logging omitted:

```java
return run(co -> {
  cleanup.run();
  co.wait(Seconds.of(1.0));
  Timer timer = Timer.createStarted();
  while (!timer.hasElapsed(10.0)) {
    runCharacterization(sign * (dynamic ? 7.0 : timer.get()));
    co.yield();
  }
  cleanup.run();
}).whenCanceled(cleanup);
```

Cancellation cleanup must use `whenCanceled` or `Command.onCancel`. Do not rely on a Java `finally` block around a suspended command to run when its coroutine is abandoned. Normal completion needs cleanup as well.

Autonomous timers, heading controllers, debouncers, and the vision-creep latch are initialized inside command execution, giving each run fresh state. Preload shooting shares one timer with its minimum delay, so the delay overlaps the shot instead of adding to it. Gathering forks current monitoring alongside path following; the speed cap is cleared on completion or cancellation. The moving-shot routine cancels rehome work before deploying the intake.

Trench shots preserve the previous policy: after a 2 s bounded aim wait, feeding proceeds even on timeout, with the outcome logged at `Auto/AimWaitTimedOut`. Moving-shot feeding still waits indefinitely for shooter readiness, so an unready shooter cannot feed. Canceling a routine cancels its children, idles shooter/indexer/intake, stops drive, and clears the trajectory speed cap. These policies are explicit in the coroutine code rather than implied by nested groups.

Choreo's supplied autonomous helpers currently use Commands v2. This project uses Choreo's trajectory loading, sampling, and transformations, with local v3 adapters under `frc.robot.auto`.

## Simulator checks

With the Alpha 7 JDK selected:

```powershell
# All automated tests
.\gradlew.bat test --console=plain

# Deterministic driving and autonomous checks
.\gradlew.bat test --tests frc.robot.SimulationFunctionalTest --console=plain

# Later shooting, gathering, moving-shot, and cancellation phases on both alliances
.\gradlew.bat test --tests frc.robot.AutonomousCoroutineTest --console=plain

# Replay those scenarios at real-time speed with the simulator GUI
.\gradlew.bat simulateChecks --console=plain
```

The GUI replay pauses for 15 seconds at startup to allow opening the Field2D views. The checks instantiate the actual robot and physics IO, feed driver-station and controller data, and run the normal lifecycle and scheduler at 20 ms intervals. Test classes use separate JVMs to isolate HAL, logger, and scheduler state.

Coverage includes:

- Teleop forward, strafe, rotation, joystick release, and disabled input on both alliances.
- Depot Cycle, Depot Inside, Safe, and mirrored Trench Depot Points on blue and red.
- Autonomous selection, alliance preview transforms, published Field2D poses, disable cancellation, and transitions back to teleop.
- Coroutine cleanup, deadline cancellation, trajectory marker lifetimes, deployed path loading, SmartIO servo mapping, robot construction, and actual PhotonVision tag observations and timestamps.
- Routine restart, bounded readiness timeout/cancellation, and shooting-to-driving transitions in Shoot Only, Safe Shoot First, Depot Double Pass, Trench Depot Points, Trench Outpost Disrupt, Trench Depot Follow, and Depot Single Pass Shoot On Move on both alliances. These later-phase checks validate progression and cleanup; they do not assert every later path against its trajectory.

Safe includes its bounded shooting phase. Trench Depot Points has trajectory comparisons for its first drive phase; its later phases have progression and cancellation checks. Path bounds are 35 cm peak and 15 cm endpoint position error, and 20° peak and 5° endpoint heading error. The verified runs ended within 7 cm; Trench Depot Points had the largest transient error, about 27 cm and 19°.

Recorded Field2D poses are written to `build/reports/simulation/field-poses.csv`. With Python and matplotlib installed, render the comparison plot using:

```powershell
python scripts/plot_simulation.py
```

The result is `build/reports/simulation/field-poses.png`. JUnit results are under `build/reports/tests/test`.

Simulation does not model obstacles, bumps, collected game pieces, or physical shooter accuracy. Its camera is ideal: 50 FPS, zero latency, and zero image noise. Camera motion uses wheel odometry independently of vision corrections. The pinned PhotonVision dev release produced an occasional invalid multi-tag solve; the simulation adapter drops implausible solves against known simulated motion and reports a rejection count. This workaround does not validate real-camera accuracy and must be revisited when adding camera noise or latency.

Shooter simulation converts Phoenix velocity gains from rotations/sec to radians/sec and implements FOC requests through the voltage-based motor model. Hardware torque-current limits and bang-bang transients are not modeled. A regression test verifies spin-up and stopping rather than physical shot accuracy.

## Telemetry and autonomous selection

Configuration uses Alpha 7 **Tunables** and outputs use **Telemetry**. Do not add the legacy SmartDashboard API or NetworkTables namespace. `Field2d` still resides in WPILib's historical `org.wpilib.smartdashboard` Java package; publishing it here uses Telemetry.

| Purpose | NetworkTables path |
| --- | --- |
| Autonomous chooser | `/Tunables/Autonomous/Chooser` |
| Select an autonomous routine | `/Tunables/Autonomous/Chooser/selected/tune` |
| Read the selected routine | `/Tunables/Autonomous/Chooser/selected/value` |
| Shoot-first delay | `/Tunables/Autonomous/ShootFirstDelaySecs` |
| Robot Field2D | `/Telemetry/Drive/Field` |
| Autonomous preview Field2D | `/Telemetry/Autonomous/Preview` |
| Rejected simulated vision solves | `/Telemetry/Vision/Simulation/RejectedSolves` |

Choose the alliance before starting autonomous. Alliance-flipped paths refuse to start with an unknown alliance. The preview updates when the selection or alliance changes.

## Architecture and hardware

| Location | Responsibility |
| --- | --- |
| `Robot.java` | AdvantageKit logging, lifecycle, scheduler, and mode transitions |
| `RobotContainer.java` | IO selection, controller bindings, autonomous selection and preview |
| `AutoRoutines.java`, `auto/` | Autonomous behavior and Commands v3 Choreo adapters |
| `subsystems/drive/` | Swerve drive, Pigeon2 gyro, and pose estimation |
| `subsystems/intake/`, `subsystems/indexer/` | Collection and feeding mechanisms |
| `subsystems/shooter/` | Flywheel, hood, and distance-based shot settings |
| `subsystems/vision/` | Limelight, PhotonVision simulation, and QuestNav IO |
| `generated/TunerConstants.java` | CTRE swerve configuration |
| `src/main/deploy/` | Choreo paths and deployed assets |

Hardware access belongs in hardware IO implementations; physics belongs in simulation IO. `Constants.currentMode` selects REAL on hardware and otherwise uses `Constants.simMode`, currently SIM. REPLAY uses an AdvantageKit log instead of live IO.

REAL uses Pigeon2 and Limelight. Hood servos plug directly into SystemCore SmartIO PWM channels **0 and 1**. The configured period is 20 ms, with pulse endpoints of 600–2400 microseconds and a normalized 20–45° angle mapping. Verify calibration with the installed servos before hardware use. Preserve the existing CAN IDs, inversions, gearing, current limits, gains, and field conventions when modifying behavior.

## Dependencies

| Dependency | Pinned version |
| --- | --- |
| WPILib / GradleRIO | `2027.0.0-alpha-7` |
| Commands v3 | Official Alpha 7 `org.wpilib:commandsv3-java` artifact |
| Phoenix 6 | `26.70.0-alpha-2` |
| REVLib | `2027.0.0-alpha-8` |
| AdvantageKit | `27.0.0-alpha-6` |
| Choreo | `2027.0.0-alpha-3` |
| PhotonLib | `dev-v2027.0.0-alpha-2-73-g4651dbae` |

Vendor alpha numbers do not need to match WPILib's alpha number. Verify compatibility against the actual artifacts and sources before upgrading. AdvantageKit `@AutoLog` classes are generated by annotation processing; do not handwrite generated inputs classes.

See [`AGENTS.md`](AGENTS.md) for detailed migration constraints and working conventions.
