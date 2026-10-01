// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double SNAKE_MAX_VELOCITY = 6.0;
  private static final double SNAKE_MAX_ACCELERATION = 15.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.ZERO, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.ZERO))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive
        .runRepeatedly(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(
                      xSupplier.getAsDouble(), ySupplier.getAsDouble());
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
              omega = Math.copySign(omega * omega, omega);
              ChassisVelocities speeds =
                  new ChassisVelocities(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;
              drive.runVelocity(
                  (speeds).toRobotRelative(isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            })
        .named("Joystick Drive");
  }

  /**
   * Field relative drive command where the robot aims at a target using PID for angular control.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return drive
        .run(
            co -> {
              // Initialize PID state
              angleController.reset(drive.getRotation().getRadians());
              while (true) {
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(), ySupplier.getAsDouble());
                double omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(), rotationSupplier.get().getRadians());
                ChassisVelocities speeds =
                    new ChassisVelocities(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                boolean isFlipped =
                    MatchState.getAlliance().isPresent()
                        && MatchState.getAlliance().get() == Alliance.RED;
                drive.runVelocity(
                    (speeds).toRobotRelative(isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
                co.yield();
              }
            })
        .named("Joystick Drive At Angle");
  }

  /**
   * Field relative drive command where the robot automatically rotates to face the direction of
   * travel.
   */
  public static Command joystickDriveSnake(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(SNAKE_MAX_VELOCITY, SNAKE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return drive
        .run(
            co -> {
              // Initialize
              angleController.reset(drive.getRotation().getRadians());
              double[] lastHeading = {drive.getRotation().getRadians()};
              while (true) {
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(), ySupplier.getAsDouble());
                double targetHeading;
                if (linearVelocity.getNorm() > 0.01) {
                  targetHeading = Math.atan2(linearVelocity.getY(), linearVelocity.getX());
                  lastHeading[0] = targetHeading;
                } else {
                  targetHeading = lastHeading[0];
                }
                double omega =
                    angleController.calculate(drive.getRotation().getRadians(), targetHeading);
                ChassisVelocities speeds =
                    new ChassisVelocities(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                boolean isFlipped =
                    MatchState.getAlliance().isPresent()
                        && MatchState.getAlliance().get() == Alliance.RED;
                drive.runVelocity(
                    (speeds).toRobotRelative(isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
                co.yield();
              }
            })
        .named("Joystick Drive Snake");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Command.sequence(
            // Reset data
            Command.noRequirements(
                    co -> {
                      velocitySamples.clear();
                      voltageSamples.clear();
                    })
                .named("Reset FF Data"),

            // Allow modules to orient
            drive
                .runRepeatedly(() -> drive.runCharacterization(0.0))
                .named("FF Orient")
                .withTimeout(Seconds.of(FF_START_DELAY)),

            // Start timer
            Command.noRequirements(co -> timer.restart()).named("FF Start Timer"),

            // Accelerate and gather data; print results on cancel
            drive
                .run(
                    co -> {
                      while (true) {
                        double voltage = timer.get() * FF_RAMP_RATE;
                        drive.runCharacterization(voltage);
                        velocitySamples.add(drive.getFFCharacterizationVelocity());
                        voltageSamples.add(voltage);
                        co.yield();
                      }
                    })
                .whenCanceled(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double kS =
                          (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                      double kV =
                          (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                      NumberFormat formatter = new DecimalFormat("#0.00000");
                      System.out.println(
                          "********** Drive FF Characterization Results **********");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                    })
                .named("FF Accelerate"))
        .withAutomaticName();
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Command.parallel(
            // Drive control sequence
            Command.sequence(
                    Command.noRequirements(co -> limiter.reset(0.0)).named("Reset Limiter"),
                    drive
                        .runRepeatedly(
                            () -> {
                              double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                              drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
                            })
                        .named("Spin Up"))
                .withAutomaticName(),

            // Measurement sequence
            Command.sequence(
                    // Wait for modules to orient
                    Command.waitFor(Seconds.of(1.0)).named("Wait For Heading"),

                    // Record starting measurement
                    Command.noRequirements(
                            co -> {
                              state.positions = drive.getWheelRadiusCharacterizationPositions();
                              state.lastAngle = drive.getRotation();
                              state.gyroDelta = 0.0;
                            })
                        .named("Record Start"),

                    // Measure gyro delta; print results on cancel
                    Command.noRequirements(
                            co -> {
                              while (true) {
                                var rotation = drive.getRotation();
                                state.gyroDelta +=
                                    Math.abs(rotation.minus(state.lastAngle).getRadians());
                                state.lastAngle = rotation;
                                co.yield();
                              }
                            })
                        .whenCanceled(
                            () -> {
                              double[] positions =
                                  drive.getWheelRadiusCharacterizationPositions();
                              double wheelDelta = 0.0;
                              for (int i = 0; i < 4; i++) {
                                wheelDelta +=
                                    Math.abs(positions[i] - state.positions[i]) / 4.0;
                              }
                              double wheelRadius =
                                  (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;
                              NumberFormat formatter = new DecimalFormat("#0.000");
                              System.out.println(
                                  "********** Wheel Radius Characterization Results **********");
                              System.out.println(
                                  "\tWheel Delta: "
                                      + formatter.format(wheelDelta)
                                      + " radians");
                              System.out.println(
                                  "\tGyro Delta: "
                                      + formatter.format(state.gyroDelta)
                                      + " radians");
                              System.out.println(
                                  "\tWheel Radius: "
                                      + formatter.format(wheelRadius)
                                      + " meters, "
                                      + formatter.format(Units.metersToInches(wheelRadius))
                                      + " inches");
                            })
                        .named("Measure Gyro Delta"))
                .withAutomaticName())
        .withAutomaticName();
  }

  /**
   * Drives the robot forward a specified distance at a moderate speed. Useful for verifying encoder
   * accuracy by measuring actual distance traveled vs. odometry.
   */
  public static Command driveForward(Drive drive, double distance) {
    double speedMetersPerSec = 1.5;

    return Command.sequence(
            // Reset odometry to origin facing forward
            drive.run(co -> drive.setPose(new Pose2d())).named("Reset Pose"),

            // Drive forward until distance reached
            drive
                .run(
                    co -> {
                      while (drive.getPose().getTranslation().getNorm() < distance) {
                        drive.runVelocity(new ChassisVelocities(speedMetersPerSec, 0.0, 0.0));
                        co.yield();
                      }
                    })
                .named("Drive Forward"),

            // Stop
            drive.run(co -> drive.stop()).named("Stop"),

            // Print results
            Command.noRequirements(
                    co -> {
                      double actual = drive.getPose().getTranslation().getNorm();
                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println("********** Drive Forward Results **********");
                      System.out.println(
                          "\tTarget: "
                              + formatter.format(distance)
                              + " m ("
                              + formatter.format(Units.metersToFeet(distance))
                              + " ft)");
                      System.out.println(
                          "\tActual: "
                              + formatter.format(actual)
                              + " m ("
                              + formatter.format(Units.metersToFeet(actual))
                              + " ft)");
                    })
                .named("Print Drive Forward Results"))
        .withAutomaticName();
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
  }
}
