package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.backend.NetworkTablesTelemetryBackend;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.util.WPIUtilJNI;

class PhotonVisionSimulationTest {
  @Test
  void cameraProducesAccurateTagPosesAndSecondsTimestamps() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    WPIUtilJNI.enableMockTime();
    WPIUtilJNI.setMockTime(RobotController.getMonotonicTime());
    TelemetryRegistry.registerBackend("/",
        new NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), "/Telemetry"));
    var tag = aprilTagLayout.getTags().getFirst();
    var tagPose = tag.getPose();
    double yaw = tagPose.getRotation().getZ();
    // Stand two metres in front of the tag, with the rear camera pointing toward it.
    var pose = new Pose2d(tagPose.getX() + 2 * Math.cos(yaw),
        tagPose.getY() + 2 * Math.sin(yaw), new Rotation2d(yaw));
    var inputs = new VisionIO.VisionIOInputs();
    int observations = 0;
    try (var camera = new VisionIOPhotonVisionSim("regression_camera", robotToCamera0, () -> pose)) {
      for (int cycle = 0; cycle < 100; cycle++) {
        SimHooks.stepTiming(0.02);
        WPIUtilJNI.setMockTime(RobotController.getMonotonicTime());
        camera.updateInputs(inputs);
        for (var observation : inputs.poseObservations) {
          observations++;
          assertTrue(inputs.connected);
          assertTrue(inputs.tagIds.length > 0);
          assertEquals(VisionIO.PoseObservationType.PHOTONVISION, observation.type());
          assertTrue(observation.tagCount() > 0);
          assertEquals(pose.getX(), observation.pose().getX(), 0.03);
          assertEquals(pose.getY(), observation.pose().getY(), 0.03);
          assertEquals(0, observation.pose().toPose2d().getRotation()
              .minus(pose.getRotation()).getRadians(), 0.03);
          assertEquals(Timer.getMonotonicTimestamp(), observation.timestamp(), 0.04);
        }
      }
      assertTrue(observations > 20, "Camera must actually generate repeated tag observations");
    } finally {
      WPIUtilJNI.disableMockTime();
      SimHooks.resumeTiming();
    }
  }
}
