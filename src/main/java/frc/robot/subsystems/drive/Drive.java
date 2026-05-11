// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import choreo.trajectory.SwerveSample;
import org.wpilib.hardware.hal.FRCNetComm.tInstances;
import org.wpilib.hardware.hal.FRCNetComm.tResourceType;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleState;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.AlertType;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
// TODO: SysIdRoutine removed from Commands V3 — reimplement with coroutines when CTRE is available
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends Mechanism {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // Tunable drive motor gains (kP/kD only, FF preserved from TunerConstants)
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("Drive/Module/DriveKP", 12.5);
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("Drive/Module/DriveKD", 0.0);

  // Heading correction tunable gains (holds heading when not commanding rotation)
  private static final LoggedTunableNumber headingCorrectionKp =
      new LoggedTunableNumber("Drive/HeadingCorrection/kP", 3.0);
  private static final LoggedTunableNumber headingCorrectionKd =
      new LoggedTunableNumber("Drive/HeadingCorrection/kD", 0.0);

  // Choreo trajectory following tunable gains
  private static final LoggedTunableNumber trajectoryKp =
      new LoggedTunableNumber("Drive/Trajectory/kP", 5.0);
  private static final LoggedTunableNumber trajectoryKd =
      new LoggedTunableNumber("Drive/Trajectory/kD", 0.0);
  private static final LoggedTunableNumber headingKp =
      new LoggedTunableNumber("Drive/Trajectory/HeadingkP", 5.0);
  private static final LoggedTunableNumber headingKd =
      new LoggedTunableNumber("Drive/Trajectory/HeadingkD", 0.0);

  // Heading correction PID controller
  private final PIDController headingCorrectionController =
      new PIDController(headingCorrectionKp.get(), 0.0, headingCorrectionKd.get());
  private Rotation2d headingTarget = null;

  // Choreo trajectory following PID controllers
  private final PIDController xController =
      new PIDController(trajectoryKp.get(), 0.0, trajectoryKd.get());
  private final PIDController yController =
      new PIDController(trajectoryKp.get(), 0.0, trajectoryKd.get());
  private final PIDController headingController =
      new PIDController(headingKp.get(), 0.0, headingKd.get());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final Field2d field2d = new Field2d();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Add Field2d to SmartDashboard for Glass visualization
    SmartDashboard.putData("Field", field2d);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure heading controllers for continuous input
    headingCorrectionController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update tunable drive motor gains
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          for (var module : modules) {
            module.setDriveGains(values[0], values[1]);
          }
        },
        driveKp,
        driveKd);

    // Update tunable heading correction gains
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> headingCorrectionController.setPID(values[0], 0.0, values[1]),
        headingCorrectionKp,
        headingCorrectionKd);

    // Update tunable PID gains
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          xController.setPID(values[0], 0.0, values[1]);
          yController.setPID(values[0], 0.0, values[1]);
        },
        trajectoryKp,
        trajectoryKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> headingController.setPID(values[0], 0.0, values[1]),
        headingKp,
        headingKd);

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Update Field2d for Glass visualization
    field2d.setRobotPose(getPose());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Heading correction: hold heading when no rotation is commanded
    /* if (Math.abs(speeds.omegaRadiansPerSecond) < 0.05) {
      if (headingTarget == null) {
        headingTarget = getRotation();
      }
      speeds.omegaRadiansPerSecond =
          headingCorrectionController.calculate(
              getRotation().getRadians(), headingTarget.getRadians());
    } else {
      headingTarget = null;
    } */

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Follows a Choreo trajectory sample using feedforward + PID feedback. */
  public void followTrajectory(SwerveSample sample) {
    Pose2d pose = getPose();

    // Compute PID corrections
    double xCorrection = xController.calculate(pose.getX(), sample.x);
    double yCorrection = yController.calculate(pose.getY(), sample.y);
    double headingCorrectionRaw =
        headingController.calculate(pose.getRotation().getRadians(), sample.heading);
    double headingCorrectionClamped = MathUtil.clamp(headingCorrectionRaw, -2.0, 2.0);

    // Field-relative speeds: feedforward from trajectory + PID feedback
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx + xCorrection,
            sample.vy + yCorrection,
            sample.omega + headingCorrectionClamped,
            pose.getRotation());

    // Apply gather clump speed cap if active: scale down translation, preserve heading correction
    if (trajectorySpeedCapMps > 0.0) {
      double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      if (linearSpeed > trajectorySpeedCapMps) {
        double scale = trajectorySpeedCapMps / linearSpeed;
        speeds =
            new ChassisSpeeds(
                speeds.vxMetersPerSecond * scale,
                speeds.vyMetersPerSecond * scale,
                speeds.omegaRadiansPerSecond);
      }
    }

    // Trajectory following telemetry (visible in AdvantageScope under Drive/Trajectory/)
    Logger.recordOutput("Odometry/TrajectorySetpoint", sample.getPose());
    Logger.recordOutput("Drive/Trajectory/XErrorMeters", sample.x - pose.getX());
    Logger.recordOutput("Drive/Trajectory/YErrorMeters", sample.y - pose.getY());
    Logger.recordOutput(
        "Drive/Trajectory/TranslationErrorMeters",
        Math.hypot(sample.x - pose.getX(), sample.y - pose.getY()));
    Logger.recordOutput(
        "Drive/Trajectory/HeadingErrorDegrees",
        Math.toDegrees(headingController.getPositionError()));
    Logger.recordOutput("Drive/Trajectory/HeadingCorrectionRawRadPerSec", headingCorrectionRaw);
    Logger.recordOutput(
        "Drive/Trajectory/HeadingCorrectionClampedRadPerSec", headingCorrectionClamped);

    runVelocity(speeds);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot (robot-relative). */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the measured chassis speeds in the field frame. */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns true if the robot heading is within toleranceDeg degrees of the target rotation. */
  public boolean isHeadingAt(Rotation2d target, double toleranceDeg) {
    double errorRad = Math.abs(MathUtil.angleModulus(getRotation().minus(target).getRadians()));
    return errorRad < Math.toRadians(toleranceDeg);
  }

  /** Returns the current pitch angle in radians from the gyro. */
  public double getPitch() {
    return gyroInputs.pitchPositionRad;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // Optional speed cap (0 = no override)
  private double maxSpeedOverrideMetersPerSec = 0.0;

  // Cap applied to translational speed during trajectory following (0 = no cap)
  private double trajectorySpeedCapMps = 0.0;

  /** Sets a maximum speed override in meters per sec. */
  public void setMaxSpeedOverride(double maxSpeedMetersPerSec) {
    this.maxSpeedOverrideMetersPerSec = maxSpeedMetersPerSec;
  }

  /** Clears the maximum speed override. */
  public void clearMaxSpeedOverride() {
    this.maxSpeedOverrideMetersPerSec = 0.0;
  }

  /**
   * Caps translational speed during trajectory following. Call clearTrajectorySpeedCap() to remove.
   */
  public void setTrajectorySpeedCap(double mps) {
    trajectorySpeedCapMps = mps;
  }

  /** Removes the trajectory speed cap. */
  public void clearTrajectorySpeedCap() {
    trajectorySpeedCapMps = 0.0;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    double baseSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    if (maxSpeedOverrideMetersPerSec > 0.0) {
      return Math.min(baseSpeed, maxSpeedOverrideMetersPerSec);
    }
    return baseSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
