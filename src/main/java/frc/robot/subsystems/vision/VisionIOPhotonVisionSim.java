// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.telemetry.Telemetry;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim implements VisionIO, AutoCloseable {
  private final VisionSystemSim visionSim;

  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private int rejectedSolveCount;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    visionSim = new VisionSystemSim(name);
    visionSim.addAprilTags(aprilTagLayout);

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setFPS(50);
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void close() {
    cameraSim.close();
    camera.close();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update simulation
    Pose2d simPose = poseSupplier.get();
    visionSim.update(simPose);

    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.ZERO, Rotation2d.ZERO);
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // A planar single tag has two PnP solutions. Perfect simulated corners can make
          // their reprojection errors indistinguishable; use the current pose as a reference.
          Transform3d alternateFieldToRobot = fieldToTarget
              .plus(target.altCameraToTarget.inverse()).plus(robotToCamera.inverse());
          Pose3d alternateRobotPose = new Pose3d(alternateFieldToRobot.getTranslation(),
              alternateFieldToRobot.getRotation());
          if (poseDistance(alternateRobotPose, simPose) < poseDistance(robotPose, simPose)) {
            robotPose = alternateRobotPose;
          }

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    // The dev OpenCV multi-tag solver occasionally returns the wrong planar solution even
    // for perfect corners. In this zero-latency ideal-camera model, a solve this far from
    // known simulated motion is invalid. Drop the frame, never replace it with ground truth.
    poseObservations.removeIf(observation -> {
      boolean invalid = poseDistance(observation.pose(), simPose) > 0.5;
      if (invalid) rejectedSolveCount++;
      return invalid;
    });
    Telemetry.log("Vision/Simulation/RejectedSolves", rejectedSolveCount);
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  private static double poseDistance(Pose3d observation, Pose2d reference) {
    return observation.toPose2d().getTranslation().getDistance(reference.getTranslation())
        + Math.abs(observation.toPose2d().getRotation().minus(reference.getRotation()).getRadians());
  }
}
