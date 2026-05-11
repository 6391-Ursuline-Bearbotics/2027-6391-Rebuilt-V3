// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.FloatArraySubscriber;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.RobotController;

/** IO implementation for Meta Quest with QuestNav. */
public class VisionIOQuestNav implements VisionIO {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  private final NetworkTable nt4Table;
  private final IntegerSubscriber questMiso;
  private final IntegerPublisher questMosi;

  // Subscribe to the Network Tables questnav data topics
  private final DoubleSubscriber questTimestamp;
  private final FloatArraySubscriber questPosition;
  private final FloatArraySubscriber questQuaternion;
  private final FloatArraySubscriber questEulerAngles;
  private final DoubleSubscriber questBatteryPercent;

  // Pose of the Quest when the pose was reset
  private Pose2d resetPoseOculus = new Pose2d();

  // Pose of the robot when the pose was reset
  private Pose2d resetPoseRobot = new Pose2d();

  // Position of the quest on the robot - adjust these values for your robot!
  private final Transform2d robotToQuest =
      new Transform2d(
          Units.inchesToMeters(-7.5),
          Units.inchesToMeters(-11),
          new Rotation2d(Units.degreesToRadians(-120)));

  /**
   * Creates a new VisionIOQuestNav.
   *
   * @param name The name for logging purposes (not used for NT, always uses "questnav" table).
   */
  public VisionIOQuestNav(String name) {
    NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    nt4Table = nt4Instance.getTable("questnav");

    questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    questMosi = nt4Table.getIntegerTopic("mosi").publish();
    questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    questPosition =
        nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questQuaternion =
        nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    questEulerAngles =
        nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

    // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = isConnected();

    // Update target observation (QuestNav doesn't provide target angles)
    inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    // Get robot pose
    var pose = getRobotPose();

    // Create pose observation
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              getTimestamp(), // Timestamp
              new Pose3d(pose), // 3D pose estimate
              0.0, // Ambiguity (not applicable for QuestNav)
              1, // Tag count (use 1 to pass filtering)
              0.0, // Average tag distance (not applicable)
              PoseObservationType.QUESTNAV)
        };

    // No AprilTag IDs for QuestNav
    inputs.tagIds = new int[0];

    cleanUpQuestNavMessages();
  }

  @Override
  public void resetPose(Pose2d newPose) {
    resetPoseOculus = getUncorrectedOculusPose().transformBy(robotToQuest.inverse());
    resetPoseRobot = newPose;
  }

  /** Gets the pose of the robot on the field. */
  public Pose2d getRobotPose() {
    return getQuestPose().transformBy(robotToQuest.inverse());
  }

  /** Gets the pose of the Quest on the field. */
  public Pose2d getQuestPose() {
    var rawPose = getUncorrectedOculusPose();
    var poseRelativeToReset = rawPose.minus(resetPoseOculus);
    return resetPoseRobot.transformBy(poseRelativeToReset);
  }

  /** Gets the battery percent of the Quest. */
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  /** Returns if the Quest is connected. */
  public boolean isConnected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  /** Gets the raw Rotation3d of the Quest. */
  public Rotation3d getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Rotation3d(new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]));
  }

  /** Gets the Quest's timestamp. */
  public double getTimestamp() {
    return questTimestamp.get();
  }

  /**
   * Clean up questnav subroutine messages after processing on the headset. Call this each iteration
   * to reset the command after it has been processed.
   */
  private void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  /** Gets the raw pose of the oculus, relative to the position where it started. */
  private Pose2d getUncorrectedOculusPose() {
    var eulerAngles = questEulerAngles.get();
    var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

    var questnavPosition = questPosition.get();
    var translation = new Translation2d(questnavPosition[2], -questnavPosition[0]);
    return new Pose2d(translation, rotation);
  }
}
