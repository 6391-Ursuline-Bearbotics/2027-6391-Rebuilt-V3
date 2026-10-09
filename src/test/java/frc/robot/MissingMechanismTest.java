package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

class MissingMechanismTest {
  @BeforeAll
  static void enable() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    org.wpilib.telemetry.TelemetryRegistry.registerBackend("/",
        new org.wpilib.telemetry.DiscardTelemetryBackend());
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setOpMode((long) RobotMode.TELEOPERATED.getValue() << 56);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStationBackend.refreshData();
  }

  private static final class Flywheels implements ShooterIO {
    boolean left = true, right = true;
    double request;
    @Override public void updateInputs(ShooterIOInputs inputs) {
      inputs.leftConnected = left;
      inputs.rightConnected = right;
      // Deliberately leave matching velocity readings when feedback disappears.
      inputs.leftVelocityRadPerSec = request;
      inputs.rightVelocityRadPerSec = request;
    }
    @Override public void setVelocity(double velocity) { request = velocity; }
    @Override public void setVelocityFOC(double velocity) { request = velocity; }
    @Override public void setVoltage(double voltage) { request = voltage; }
    @Override public void stop() { request = 0; }
  }

  @Test
  void missingFlywheelNeverLooksReadyAndStopsTheRemainingMotor() {
    var io = new Flywheels();
    class Hood implements ShooterHoodIO {
      boolean available = true;
      double angle;
      @Override public void updateInputs(ShooterHoodIOInputs inputs) {
        inputs.outputAvailable = available;
        inputs.positionDeg = angle;
      }
      @Override public void setAngle(double value) { angle = value; }
    }
    var hood = new Hood();
    var shooter = new Shooter(io, hood, () -> new Pose2d(3, 4,
        org.wpilib.math.geometry.Rotation2d.ZERO), () -> new ChassisVelocities(),
        () -> false, () -> 0.0, () -> false);
    shooter.setGoal(Shooter.Goal.PASS);
    shooter.periodic();
    shooter.periodic();
    assertTrue(shooter.isAvailable());
    assertTrue(shooter.isAtSetpoint());
    io.right = false;
    shooter.periodic();
    assertFalse(shooter.isAvailable());
    assertFalse(shooter.isAtSetpoint());
    assertEquals(0, shooter.getCommandedRPM());
    assertEquals(0, io.request);
    shooter.runCharacterization(6);
    assertEquals(0, io.request, "Characterization must honor the same interlock");
    io.right = true;
    shooter.periodic();
    assertTrue(shooter.isAvailable());
    assertTrue(io.request > 0, "Valid feedback permits the retained request again");
    hood.available = false;
    shooter.periodic();
    assertFalse(shooter.isHoodOutputAvailable());
    assertFalse(shooter.isHoodAtAngle(hood.angle, 1));
    assertFalse(shooter.isHoodAtOrBelow26Deg());
    assertFalse(shooter.isAvailable());
  }

  @Test
  void missingIntakeDeviceStopsBothOutputsWithoutInferringAHardStop() {
    class Deploy implements IntakeDeployIO {
      boolean connected = true;
      double volts;
      @Override public void updateInputs(IntakeDeployIOInputs inputs) {
        inputs.connected = connected;
        inputs.statorCurrentAmps = connected ? 0 : 100; // Stale current cannot establish position.
      }
      @Override public void setVoltage(double value) { volts = value; }
      @Override public void stop() { volts = 0; }
    }
    class Roller implements IntakeRollerIO {
      double volts;
      @Override public void updateInputs(IntakeRollerIOInputs inputs) { inputs.connected = true; }
      @Override public void setVoltage(double value) { volts = value; }
      @Override public void stop() { volts = 0; }
    }
    var deploy = new Deploy();
    var roller = new Roller();
    var intake = new Intake(deploy, roller);
    intake.setGoal(Intake.Goal.INTAKE);
    intake.periodic();
    assertTrue(intake.isAvailable());
    assertNotEquals(0, deploy.volts);
    deploy.connected = false;
    SimHooks.stepTiming(1);
    intake.periodic();
    assertFalse(intake.isAvailable());
    assertFalse(intake.isDeployed());
    assertFalse(intake.isRetracted());
    assertEquals(0, deploy.volts);
    assertEquals(0, roller.volts);
    intake.runRollerCharacterization(6);
    assertEquals(0, roller.volts);
    deploy.connected = true;
    intake.periodic();
    assertTrue(intake.isAvailable());
    assertFalse(intake.isDeployed(), "Recovery must reacquire a hard stop with fresh feedback");
  }

  @Test
  void missingBeltOrShooterInhibitsFeedingButOptionalSpinnersDoNotDisableTheIndexer() {
    class Belt implements IndexerBeltIO {
      boolean connected = true;
      double velocity;
      @Override public void updateInputs(IndexerBeltIOInputs inputs) { inputs.connected = connected; }
      @Override public void setVelocity(double value) { velocity = value; }
      @Override public void stop() { velocity = 0; }
    }
    class Kicker implements IndexerKickerIO {
      double velocity;
      @Override public void updateInputs(IndexerKickerIOInputs inputs) { inputs.connected = true; }
      @Override public void setVelocity(double value) { velocity = value; }
      @Override public void stop() { velocity = 0; }
    }
    var belt = new Belt();
    var kicker = new Kicker();
    var indexer = new Indexer(belt, kicker, new SpinnersIO() {}, Pose2d::new);
    indexer.setGoal(Indexer.Goal.EJECT);
    indexer.periodic();
    assertTrue(indexer.isAvailable());
    assertNotEquals(0, kicker.velocity);
    belt.connected = false;
    indexer.periodic();
    assertFalse(indexer.isAvailable());
    assertEquals(0, belt.velocity);
    assertEquals(0, kicker.velocity);
    belt.connected = true;
    indexer.setFeedAvailableSupplier(() -> false);
    indexer.setGoal(Indexer.Goal.FEED);
    indexer.periodic();
    assertTrue(indexer.isAvailable());
    assertEquals(0, belt.velocity);
    assertEquals(0, kicker.velocity);
    indexer.setGoal(Indexer.Goal.EJECT);
    indexer.periodic();
    assertNotEquals(0, kicker.velocity, "Ejection remains available without a shooter");
  }
  @Test
  void disconnectedCameraCannotReuseCachedPosesOrTags() {
    class Camera implements frc.robot.subsystems.vision.VisionIO {
      boolean connected = true;
      @Override public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = connected;
        inputs.tagIds = new int[] {1, 2};
        inputs.poseObservations = new PoseObservation[] {new PoseObservation(
            org.wpilib.system.Timer.getTimestamp(), new org.wpilib.math.geometry.Pose3d(
                3, 4, 0, org.wpilib.math.geometry.Rotation3d.ZERO), 0, 2, 2,
            PoseObservationType.PHOTONVISION)};
      }
    }
    var camera = new Camera();
    var admitted = new java.util.concurrent.atomic.AtomicInteger();
    var vision = new frc.robot.subsystems.vision.Vision(
        (pose, time, deviations) -> admitted.incrementAndGet(), camera);
    vision.periodic();
    assertEquals(1, admitted.get());
    camera.connected = false;
    SimHooks.stepTiming(0.6);
    vision.periodic();
    assertEquals(1, admitted.get(), "Disconnected camera must not correct odometry");
    assertFalse(vision.hasCameraConnected());
    assertFalse(vision.hasTagsInView());
  }

}
