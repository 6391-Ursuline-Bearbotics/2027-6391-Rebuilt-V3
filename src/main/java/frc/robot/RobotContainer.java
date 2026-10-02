// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static org.wpilib.units.Units.Seconds;

import choreo.Choreo;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.tunable.Tunables;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.command3.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerBeltIO;
import frc.robot.subsystems.indexer.IndexerBeltIOSim;
import frc.robot.subsystems.indexer.IndexerBeltIOTalonFX;
import frc.robot.subsystems.indexer.IndexerKickerIO;
import frc.robot.subsystems.indexer.IndexerKickerIOSim;
import frc.robot.subsystems.indexer.IndexerKickerIOTalonFX;
import frc.robot.subsystems.indexer.SpinnersIO;
import frc.robot.subsystems.indexer.SpinnersIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeDeployIO;
import frc.robot.subsystems.intake.IntakeDeployIOSim;
import frc.robot.subsystems.intake.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterHoodIO;
import frc.robot.subsystems.shooter.ShooterHoodIOServo;
import frc.robot.subsystems.shooter.ShooterHoodIOSim;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Drive modes
  public enum DriveMode {
    STANDARD,
    AIM_TARGET // Aims at Hub in alliance zone, passing target otherwise
  }

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  // Set to false to use no-op IO when hardware is not connected
  private static final boolean indexerEnabled = true;
  private static final boolean shooterEnabled = true;

  // Controllers
  private final CommandNiDsXboxController drv = new CommandNiDsXboxController(0);
  private final CommandNiDsXboxController op = new CommandNiDsXboxController(1);

  // Auto
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser;
  private final Field2d autoPreviewField = new Field2d();
  private String lastPreviewName = "";
  private boolean lastPreviewRed;

  // Current drive mode
  private DriveMode currentDriveMode = DriveMode.STANDARD;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));

        intake = new Intake(new IntakeDeployIOTalonFX(), new IntakeRollerIOTalonFX());

        indexer =
            indexerEnabled
                ? new Indexer(
                    new IndexerBeltIOTalonFX(),
                    new IndexerKickerIOTalonFX(),
                    new SpinnersIOSparkMax(),
                    drive::getPose)
                : new Indexer(
                    new IndexerBeltIO() {},
                    new IndexerKickerIO() {},
                    new SpinnersIO() {},
                    drive::getPose);

        shooter =
            shooterEnabled
                ? new Shooter(
                    new ShooterIOTalonFX(),
                    new ShooterHoodIOServo(),
                    drive::getPose,
                    drive::getFieldRelativeSpeeds,
                    () -> indexer.getGoal() == Indexer.Goal.FEED,
                    drive::getPitch,
                    drv.leftTrigger(0.5)
                        .or(op.rightTrigger(0.5))
                        .or(op.leftTrigger(0.5))
                        .or(op.leftBumper()))
                : new Shooter(
                    new ShooterIO() {},
                    new ShooterHoodIO() {},
                    drive::getPose,
                    drive::getFieldRelativeSpeeds,
                    () -> indexer.getGoal() == Indexer.Goal.FEED,
                    drive::getPitch,
                    drv.leftTrigger(0.5)
                        .or(op.rightTrigger(0.5))
                        .or(op.leftTrigger(0.5))
                        .or(op.leftBumper()));

        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {});
        new org.wpilib.util.Alert(
            "PhotonVisionSimulationUnavailable",
            "Camera simulation disabled until a compatible PhotonVision vendordep is installed.",
            org.wpilib.util.Alert.Level.LOW).set(true);
        intake = new Intake(new IntakeDeployIOSim(), new IntakeRollerIOSim());
        indexer =
            new Indexer(
                new IndexerBeltIOSim(),
                new IndexerKickerIOSim(),
                new SpinnersIO() {},
                drive::getPose);
        shooter =
            new Shooter(
                new ShooterIOSim(),
                new ShooterHoodIOSim(),
                drive::getPose,
                drive::getFieldRelativeSpeeds,
                () -> indexer.getGoal() == Indexer.Goal.FEED,
                drive::getPitch,
                drv.rightTrigger(0.5)
                    .or(op.rightTrigger(0.5))
                    .or(op.leftTrigger(0.5))
                    .or(op.leftBumper()));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        intake = new Intake(new IntakeDeployIO() {}, new IntakeRollerIO() {});
        indexer =
            new Indexer(
                new IndexerBeltIO() {},
                new IndexerKickerIO() {},
                new SpinnersIO() {},
                drive::getPose);
        shooter =
            new Shooter(
                new ShooterIO() {},
                new ShooterHoodIO() {},
                drive::getPose,
                drive::getFieldRelativeSpeeds,
                () -> indexer.getGoal() == Indexer.Goal.FEED,
                drive::getPitch,
                () -> false);
        break;
    }

    // Set up Choreo auto factory, routines, and chooser
    autoFactory =
        new AutoFactory(drive::setPose, drive::followTrajectory, true, drive);

    // Preload Choreo classes at init time instead of delaying auto start
    Choreo.<SwerveSample>loadTrajectory("Safe");

    autoRoutines = new AutoRoutines(autoFactory, drive, intake, indexer, shooter, vision);
    autoChooser = new AutoChooser();
    Tunables.getTable("Autonomous").publish("Chooser", autoChooser);
    org.wpilib.telemetry.Telemetry.log("Autonomous/Preview", autoPreviewField);

    // Competition auto routines (always available)
    autoChooser.addRoutine("Shoot Only", autoRoutines::shootOnly);
    autoChooser.addRoutine("Safe", autoRoutines::safe);
    autoChooser.addRoutine("Safe (Shoot First)", autoRoutines::safeShootFirst);
    autoChooser.addRoutine("Outpost Double Pass", autoRoutines::outpostDoublePass);
    autoChooser.addRoutine(
        "Outpost Double Pass (Shoot First)", autoRoutines::outpostDoublePassShootFirst);
    autoChooser.addRoutine("Outpost Single Pass", autoRoutines::outpostSinglePass);
    autoChooser.addRoutine(
        "Outpost Single Pass (Shoot First)", autoRoutines::outpostSinglePassShootFirst);
    autoChooser.addRoutine("Outpost FULL Pass", autoRoutines::outpostFullPass);
    autoChooser.addRoutine("Trench Outpost Disrupt", autoRoutines::trenchOutpostDisrupt);
    autoChooser.addRoutine("Trench Outpost Points", autoRoutines::trenchOutpostPoints);
    autoChooser.addRoutine("Trench Outpost Follow", autoRoutines::trenchOutpostFollow);
    autoChooser.addRoutine("Depot Double Pass", autoRoutines::depotDoublePass);
    autoChooser.addRoutine(
        "Depot Double Pass (Shoot First)", autoRoutines::depotDoublePassShootFirst);
    autoChooser.addRoutine("Depot Single Pass", autoRoutines::depotSinglePass);
    autoChooser.addRoutine(
        "Depot Single Pass (Shoot First)", autoRoutines::depotSinglePassShootFirst);
    autoChooser.addRoutine(
        "Depot Single Pass Shoot On Move", autoRoutines::depotSinglePassShootOnMove);
    autoChooser.addRoutine("Trench Depot Disrupt", autoRoutines::trenchDepotDisrupt);
    autoChooser.addRoutine("Trench Depot Points", autoRoutines::trenchDepotPoints);
    autoChooser.addRoutine("Trench Depot Follow", autoRoutines::trenchDepotFollow);

    // Test and characterization routines (tuning mode only)
    if (Constants.tuningMode) {
      autoChooser.addRoutine("Reset Odometry Start", autoRoutines::resetOdometryStart);
      autoChooser.addRoutine("Depot Cycle", autoRoutines::depotCycle);
      autoChooser.addRoutine("Depot Inside", autoRoutines::depotInside);
      autoChooser.addCmd(
          "Drive Wheel Radius Characterization",
          () -> DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addCmd(
          "Drive Simple FF Characterization",
          () -> DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addCmd(
          "Drive Forward 15ft", () -> DriveCommands.driveForward(drive, Units.feetToMeters(15)));
      autoChooser.addCmd("Drive SysId (Quasistatic Forward)",
          () -> drive.sysIdQuasistatic(Drive.SysIdDirection.FORWARD));
      autoChooser.addCmd("Drive SysId (Quasistatic Reverse)",
          () -> drive.sysIdQuasistatic(Drive.SysIdDirection.REVERSE));
      autoChooser.addCmd("Drive SysId (Dynamic Forward)",
          () -> drive.sysIdDynamic(Drive.SysIdDirection.FORWARD));
      autoChooser.addCmd("Drive SysId (Dynamic Reverse)",
          () -> drive.sysIdDynamic(Drive.SysIdDirection.REVERSE));
      autoChooser.addCmd(
          "Intake Roller FF Characterization", () -> Intake.rollerFFCharacterization(intake));
      autoChooser.addCmd(
          "Shooter FF Characterization", () -> Shooter.shooterFFCharacterization(shooter));
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command: dispatches to current drive mode each loop cycle
    drive.setDefaultCommand(
        drive.runRepeatedly(
                () -> {
                  if (!RobotState.isTeleopEnabled()) {
                    drive.stop();
                    return;
                  }
                  switch (currentDriveMode) {
                    case STANDARD:
                      runStandardDrive();
                      break;
                    case AIM_TARGET:
                      runAimTargetDrive();
                      break;
                  }
                })
            .named("Drive Default"));

    // B: return to standard drive mode
    drv.b()
        .onTrue(
            Command.noRequirements(co -> currentDriveMode = DriveMode.STANDARD)
                .named("Set Standard Mode"));

    // Y (held): drive normally if joystick active, otherwise lock wheels in X
    drv.y()
        .whileTrue(
            drive.runRepeatedly(
                    () -> {
                      double lx = MathUtil.applyDeadband(drv.getLeftX(), 0.1);
                      double ly = MathUtil.applyDeadband(drv.getLeftY(), 0.1);
                      if (Math.hypot(lx, ly) > 0.0) {
                        runStandardDrive();
                      } else {
                        drive.stopWithX();
                      }
                    })
                .named("Drive Or Stop With X"));

    // A: switch to aim-target drive mode and spin up shooter
    drv.a()
        .onTrue(
            Command.noRequirements(
                    co -> {
                      currentDriveMode = DriveMode.AIM_TARGET;
                      aimTargetController.reset(drive.getRotation().getRadians());
                      shooter.setGoal(Shooter.Goal.SHOOT);
                    })
                .named("Set Aim Target"));

    // X: spin up shooter, aim at hub for 1s, then lock wheels in X
    drv.x()
        .onTrue(
            Command.sequence(
                    Command.noRequirements(co -> shooter.setGoal(Shooter.Goal.SHOOT))
                        .named("Spin Up Shooter"),
                    aimAtHub().withTimeout(Seconds.of(1.0)),
                    drive.run(co -> drive.stopWithX()).named("Lock X"))
                .withAutomaticName());

    // Start: reset gyro yaw to zero (field-forward)
    drv.start()
        .onTrue(
            drive.run(
                    co ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.ZERO)))
                .named("Reset Gyro"));

    // Right bumper: idle shooter and move hood to 26°, wait for arrival
    drv.rightBumper()
        .onTrue(
            Command.sequence(
                    Command.noRequirements(
                            co -> {
                              shooter.setGoal(Shooter.Goal.IDLE);
                              shooter.setHoodAngle(26.0);
                            })
                        .named("Idle Shooter"),
                    Command.waitUntil(() -> shooter.isHoodAtAngle(26.0, 1.0)).named("Wait For Hood"))
                .withAutomaticName());

    // Operator intake controls
    op.a()
        .onTrue(
            Command.noRequirements(co -> intake.setGoal(Intake.Goal.INTAKE)).named("Intake"));
    op.b()
        .onTrue(
            Command.noRequirements(co -> intake.setGoal(Intake.Goal.IDLE)).named("Idle Intake"));
    op.y()
        .onTrue(
            Command.noRequirements(co -> intake.setGoal(Intake.Goal.DEPLOYED_IDLE))
                .named("Deploy Idle"));

    // Driver left bumper (held): clump intake mode at reduced speed
    drv.leftBumper()
        .whileTrue(
            Command.noRequirements(
                    co -> {
                      intake.setGoal(Intake.Goal.CLUMP_INTAKE);
                      drive.setMaxSpeedOverride(2.0);
                      co.park();
                    })
                .whenCanceled(
                    () -> {
                      intake.setGoal(Intake.Goal.INTAKE);
                      drive.clearMaxSpeedOverride();
                    })
                .named("Clump Intake"));

    // Operator left trigger (held): ungated feed + auto-spinup + adaptive intake rehome
    op.leftTrigger(0.5)
        .whileTrue(
            Command.noRequirements(
                    co -> {
                      boolean isRed =
                          MatchState.getAlliance().isPresent()
                              && MatchState.getAlliance().get() == Alliance.RED;
                      boolean inAllianceZone =
                          FieldConstants.isInOwnAllianceZone(
                              drive.getPose().getTranslation(), isRed);
                      shooter.setGoal(inAllianceZone ? Shooter.Goal.SHOOT : Shooter.Goal.PASS);
                      indexer.setGoal(Indexer.Goal.FEED);
                      co.park();
                    })
                .whenCanceled(() -> indexer.setGoal(Indexer.Goal.IDLE))
                .named("Op Left Trigger Feed")
                .alongWith(intakeWithMotionAdaptiveRehome())
                .withAutomaticName());

    // Operator left bumper (held): gated auto shot — aim + shooter ready, then feed
    op.leftBumper()
        .whileTrue(
            Command.noRequirements(
                    co -> {
                      shooter.setGoal(Shooter.Goal.SHOOT);
                      currentDriveMode = DriveMode.AIM_TARGET;
                      aimTargetController.reset(drive.getRotation().getRadians());
                      intake.setGoal(Intake.Goal.INTAKE);
                      while (!shooter.isAtSetpoint() || !isAimedAtTarget()) {
                        co.yield();
                      }
                      indexer.setGoal(Indexer.Goal.FEED);
                      co.park();
                    })
                .whenCanceled(
                    () -> {
                      currentDriveMode = DriveMode.STANDARD;
                      autoAimGyrating = false;
                      intake.setGoal(Intake.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                    })
                .named("Gated Auto Shot"));

    // Operator right bumper: stop shooter and return to standard drive
    op.rightBumper()
        .onTrue(
            Command.noRequirements(
                    co -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      currentDriveMode = DriveMode.STANDARD;
                    })
                .named("Stop Shooter"));

    // Operator right trigger OR driver left trigger (held): auto-aim + wait for ready + feed
    op.rightTrigger(0.5)
        .or(drv.leftTrigger(0.5))
        .whileTrue(
            Command.noRequirements(
                    co -> {
                      boolean isRed =
                          MatchState.getAlliance().isPresent()
                              && MatchState.getAlliance().get() == Alliance.RED;
                      boolean inAllianceZone =
                          FieldConstants.isInOwnAllianceZone(
                              drive.getPose().getTranslation(), isRed);
                      shooter.setGoal(inAllianceZone ? Shooter.Goal.SHOOT : Shooter.Goal.PASS);
                      currentDriveMode = DriveMode.AIM_TARGET;
                      aimTargetController.reset(drive.getRotation().getRadians());
                      while (!shooter.isAtSetpoint() || !isAimedAtTarget()) {
                        co.yield();
                      }
                      indexer.setGoal(Indexer.Goal.FEED);
                      co.park();
                    })
                .whenCanceled(
                    () -> {
                      currentDriveMode = DriveMode.STANDARD;
                      autoAimGyrating = false;
                      indexer.setGoal(Indexer.Goal.IDLE);
                    })
                .named("Auto Aim and Feed")
                .alongWith(rehomeOnly())
                .withAutomaticName());

    // DPAD: manual distance setpoint + spin up shooter
    op.getHID().povLeft()
        .onTrue(
            Command.noRequirements(
                    co -> {
                      shooter.setDistanceSetpoint(Units.feetToMeters(5.0));
                      shooter.setGoal(Shooter.Goal.SHOOT);
                    })
                .named("Set 5ft"));
    op.getHID().povRight()
        .onTrue(
            Command.noRequirements(
                    co -> {
                      shooter.setDistanceSetpoint(Units.feetToMeters(10.0));
                      shooter.setGoal(Shooter.Goal.SHOOT);
                    })
                .named("Set 10ft"));
    op.getHID().povUp()
        .onTrue(
            Command.noRequirements(
                    co -> {
                      shooter.adjustDistanceSetpoint(Units.feetToMeters(0.5));
                      shooter.setGoal(Shooter.Goal.SHOOT);
                    })
                .named("Adjust Distance Up"));
    op.getHID().povDown()
        .onTrue(
            Command.noRequirements(
                    co -> {
                      shooter.setHoodAngle(26.0);
                      shooter.setGoal(Shooter.Goal.IDLE);
                    })
                .named("Set Hood 26"));

    // Operator X (held): eject — reverse indexer and intake, restore on release
    op.x()
        .whileTrue(
            Command.noRequirements(
                    co -> {
                      intakeGoalBeforeEject = intake.getGoal();
                      intake.setGoal(Intake.Goal.EJECT);
                      co.park();
                    })
                .whenCanceled(() -> intake.setGoal(intakeGoalBeforeEject))
                .named("Eject Intake")
                .alongWith(indexer.ejectCommand())
                .withAutomaticName());

    // Rumble both controllers when hub activation is imminent
    new Trigger(() -> GameData.isHubActivatingSoon(5.0))
        .whileTrue(
            Command.noRequirements(
                    co -> {
                      drv.getHID().setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 1.0);
                      drv.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1.0);
                      op.getHID().setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 1.0);
                      op.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1.0);
                      co.park();
                    })
                .whenCanceled(
                    () -> {
                      drv.getHID().setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
                      drv.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
                      op.getHID().setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
                      op.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
                    })
                .named("Hub Activation Rumble"));

    // Rumble right side for 2 seconds on roller jam
    new Trigger(intake::isRollerJammed)
        .onTrue(
            Command.noRequirements(
                    co -> {
                      drv.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1.0);
                      op.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1.0);
                      org.wpilib.system.Timer t = new org.wpilib.system.Timer();
                      t.restart();
                      while (!t.hasElapsed(2.0)) {
                        co.yield();
                      }
                      drv.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
                      op.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
                    })
                .named("Roller Jam Rumble"));

    // Driver DPAD: snap intake (front) to cardinal field directions
    // UP = 180°, RIGHT = 90°, DOWN = 0°, LEFT = -90°
    final double kDpadSnapTolerance = Math.toRadians(2.0);
    drv.getHID().povUp()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -drv.getLeftY(),
                    () -> -drv.getLeftX(),
                    () -> Rotation2d.fromDegrees(180))
                .until(
                    () ->
                        Math.abs(
                                drive.getRotation().minus(Rotation2d.fromDegrees(180)).getRadians())
                            < kDpadSnapTolerance)
                .withAutomaticName());
    drv.getHID().povRight()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -drv.getLeftY(),
                    () -> -drv.getLeftX(),
                    () -> Rotation2d.fromDegrees(90))
                .until(
                    () ->
                        Math.abs(
                                drive.getRotation().minus(Rotation2d.fromDegrees(90)).getRadians())
                            < kDpadSnapTolerance)
                .withAutomaticName());
    drv.getHID().povDown()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -drv.getLeftY(),
                    () -> -drv.getLeftX(),
                    () -> Rotation2d.ZERO)
                .until(
                    () ->
                        Math.abs(drive.getRotation().minus(Rotation2d.ZERO).getRadians())
                            < kDpadSnapTolerance)
                .withAutomaticName());
    drv.getHID().povLeft()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -drv.getLeftY(),
                    () -> -drv.getLeftX(),
                    () -> Rotation2d.fromDegrees(-90))
                .until(
                    () ->
                        Math.abs(
                                drive.getRotation().minus(Rotation2d.fromDegrees(-90)).getRadians())
                            < kDpadSnapTolerance)
                .withAutomaticName());
  }

  // Intake goal saved before an eject so it can be restored on release
  private Intake.Goal intakeGoalBeforeEject = Intake.Goal.IDLE;

  // Lateral gyration during auto-aim shoot
  private boolean autoAimGyrating = false;
  private static final LoggedTunableNumber gyrationAmplitudeInches =
      new LoggedTunableNumber("Shooter/GyrationAmplitudeInches", 0.50);
  private static final LoggedTunableNumber gyrationFreqHz =
      new LoggedTunableNumber("Shooter/GyrationFreqHz", 5.0);

  // Drive mode helper methods
  private final ProfiledPIDController aimTargetController =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));

  {
    aimTargetController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns a command that runs periodic brief IDLE rehome pulses when stationary, without changing
   * the intake goal while moving.
   */
  private Command rehomeOnly() {
    org.wpilib.system.Timer rehomeTimer = new org.wpilib.system.Timer();
    boolean[] rehoming = {false};
    return Command.noRequirements(
            co -> {
              rehomeTimer.restart();
              rehoming[0] = false;
              while (true) {
                ChassisVelocities speeds = drive.getFieldRelativeSpeeds();
                double speed = Math.hypot(speeds.vx, speeds.vy);
                if (speed > 0.15) {
                  rehoming[0] = false;
                  rehomeTimer.restart();
                } else if (!rehoming[0] && rehomeTimer.hasElapsed(2.0)) {
                  intake.setGoal(Intake.Goal.IDLE);
                  rehomeTimer.restart();
                  rehoming[0] = true;
                } else if (rehoming[0] && rehomeTimer.hasElapsed(0.75)) {
                  intake.setGoal(Intake.Goal.INTAKE);
                  rehoming[0] = false;
                  rehomeTimer.restart();
                }
                co.yield();
              }
            })
        .named("Rehome Only");
  }

  /**
   * Returns a command that keeps the intake in INTAKE while moving, and runs periodic brief IDLE
   * rehome cycles when stationary.
   */
  private Command intakeWithMotionAdaptiveRehome() {
    org.wpilib.system.Timer rehomeTimer = new org.wpilib.system.Timer();
    boolean[] rehoming = {false};
    return Command.noRequirements(
            co -> {
              rehomeTimer.restart();
              rehoming[0] = false;
              intake.setGoal(Intake.Goal.INTAKE);
              while (true) {
                ChassisVelocities speeds = drive.getFieldRelativeSpeeds();
                double speed = Math.hypot(speeds.vx, speeds.vy);
                if (speed > 0.15) {
                  intake.setGoal(Intake.Goal.INTAKE);
                  rehoming[0] = false;
                  rehomeTimer.restart();
                } else if (!rehoming[0] && rehomeTimer.hasElapsed(2.0)) {
                  intake.setGoal(Intake.Goal.IDLE);
                  rehomeTimer.restart();
                  rehoming[0] = true;
                } else if (rehoming[0] && rehomeTimer.hasElapsed(0.75)) {
                  intake.setGoal(Intake.Goal.INTAKE);
                  rehoming[0] = false;
                  rehomeTimer.restart();
                }
                co.yield();
              }
            })
        .named("Intake With Rehome");
  }

  private void runStandardDrive() {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();
    double omega = MathUtil.applyDeadband(-drv.getRightX(), 0.1);
    omega = Math.copySign(omega * omega, omega);

    ChassisVelocities speeds =
        new ChassisVelocities(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec());

    boolean isFlipped =
        MatchState.getAlliance().isPresent()
            && MatchState.getAlliance().get() == Alliance.RED;
    ChassisVelocities robotRelative =
        (speeds).toRobotRelative(isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

    if (autoAimGyrating) {
      double ampM = Units.inchesToMeters(gyrationAmplitudeInches.get());
      double freqHz = gyrationFreqHz.get();
      robotRelative.vy +=
          ampM
              * 2.0
              * Math.PI
              * freqHz
              * Math.cos(2.0 * Math.PI * freqHz * Timer.getTimestamp());
    }

    drive.runVelocity(robotRelative);
  }

  private void runAimTargetDrive() {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();

    boolean isRedAlliance =
        MatchState.getAlliance().isPresent()
            && MatchState.getAlliance().get() == Alliance.RED;

    Translation2d robotPosition = drive.getPose().getTranslation();

    Translation2d target;
    if (FieldConstants.isInOwnAllianceZone(robotPosition, isRedAlliance)) {
      target = FieldConstants.getHubCenter(isRedAlliance);
    } else {
      target = FieldConstants.getPassingTarget(robotPosition, isRedAlliance);
    }

    Translation2d robotToTarget = target.minus(robotPosition);
    double targetHeading =
        Math.atan2(robotToTarget.getY(), robotToTarget.getX())
            + Math.PI
            + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);

    double omega = aimTargetController.calculate(drive.getRotation().getRadians(), targetHeading);

    double aimMaxSpeed = 1.5;
    ChassisVelocities speeds =
        new ChassisVelocities(
            linearVelocity.getX() * aimMaxSpeed, linearVelocity.getY() * aimMaxSpeed, omega);

    Rotation2d robotHeading =
        isRedAlliance ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
    ChassisVelocities robotRelative = (speeds).toRobotRelative(robotHeading);

    if (autoAimGyrating) {
      double ampM = Units.inchesToMeters(gyrationAmplitudeInches.get());
      double freqHz = gyrationFreqHz.get();
      robotRelative.vy +=
          ampM
              * 2.0
              * Math.PI
              * freqHz
              * Math.cos(2.0 * Math.PI * freqHz * Timer.getTimestamp());
    }

    drive.runVelocity(robotRelative);
  }

  private static final double kAimToleranceRad = Math.toRadians(4.5);

  private boolean isAimedAtTarget() {
    boolean isRedAlliance =
        MatchState.getAlliance().isPresent()
            && MatchState.getAlliance().get() == Alliance.RED;
    Translation2d robotPosition = drive.getPose().getTranslation();
    Translation2d target;
    if (FieldConstants.isInOwnAllianceZone(robotPosition, isRedAlliance)) {
      target = FieldConstants.getHubCenter(isRedAlliance);
    } else {
      target = FieldConstants.getPassingTarget(robotPosition, isRedAlliance);
    }
    Translation2d robotToTarget = target.minus(robotPosition);
    double targetHeading =
        Math.atan2(robotToTarget.getY(), robotToTarget.getX())
            + Math.PI
            + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
    double error = MathUtil.angleModulus(targetHeading - drive.getRotation().getRadians());
    boolean aimed = Math.abs(error) < kAimToleranceRad;
    Logger.recordOutput("Shooter/AimErrorDeg", Math.toDegrees(error));
    Logger.recordOutput("Shooter/AimedAtTarget", aimed);
    return aimed;
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    double x = -drv.getLeftY();
    double y = -drv.getLeftX();
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.ZERO, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.ZERO))
        .getTranslation();
  }

  /** Rotate in place to aim back of robot at hub. Initialization is inlined into the coroutine. */
  private Command aimAtHub() {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return drive
        .run(
            co -> {
              headingController.reset(drive.getRotation().getRadians());
              while (true) {
                Pose2d current = drive.getPose();
                boolean isRed =
                    MatchState.getAlliance().isPresent()
                        && MatchState.getAlliance().get() == Alliance.RED;
                Translation2d hubCenter = FieldConstants.getHubCenter(isRed);
                Translation2d toHub = hubCenter.minus(current.getTranslation());
                double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
                double targetHeading =
                    angleToHub
                        + Math.PI
                        + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
                double omega =
                    headingController.calculate(current.getRotation().getRadians(), targetHeading);
                drive.runVelocity(
                    new ChassisVelocities(0, 0, omega).toRobotRelative(current.getRotation()));
                co.yield();
              }
            })
        .named("Aim At Hub");
  }

  /** Called at the start of teleop to reset subsystem states coming out of auto. */
  public void teleopInit() {
    shooter.setGoal(Shooter.Goal.IDLE);
    indexer.setGoal(Indexer.Goal.IDLE);
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  public AutoFactory getAutoFactory() {
    return autoFactory;
  }

  Drive getDrive() {
    return drive;
  }

  public String getSelectedAutoName() {
    return autoChooser.selectedName();
  }

  /**
   * Publishes the final pose for this scheduler cycle and redraws the autonomous preview when
   * selection or alliance changes.
   */
  public void updateFieldVisualizations() {
    drive.publishField();
    String name = autoChooser.selectedName();
    boolean red = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
    if (name.equals(lastPreviewName) && red == lastPreviewRed) return;
    lastPreviewName = name;
    lastPreviewRed = red;
    autoPreviewField.getObject("path").setPoses(buildPreviewPoses(name));
    org.wpilib.telemetry.Telemetry.log("Autonomous/Preview", autoPreviewField);
  }

  private record TrajEntry(String name, boolean flip) {}

  private static TrajEntry t(String name) {
    return new TrajEntry(name, false);
  }

  private static TrajEntry m(String name) {
    return new TrajEntry(name, true);
  }

  @SuppressWarnings("java:S1479")
  private List<TrajEntry> buildPreviewEntries(String autoName) {
    return switch (autoName) {
      case "Depot Cycle" -> List.of(t("DepotCycle"));
      case "Depot Inside" -> List.of(t("DepotInside"));
      case "Safe", "Safe (Shoot First)" -> List.of(t("Safe"));
      case "Outpost Double Pass", "Outpost Double Pass (Shoot First)" -> List.of(
          t("OutpostBump"),
          t("OutpostDoublePass"),
          t("OutpostBumpReturn"),
          t("OutpostStagingGather"));
      case "Outpost Single Pass", "Outpost Single Pass (Shoot First)" -> List.of(
          t("OutpostBump"),
          t("OutpostSinglePass"),
          t("OutpostBumpReturn"),
          t("OutpostStagingGather"));
      case "Outpost FULL Pass" -> List.of(
          t("OutpostBump"),
          t("FullOutpostSinglePass"),
          t("OutpostBumpReturn"),
          t("OutpostStagingGather"));
      case "Trench Outpost Disrupt" -> List.of(
          t("TrenchOutpostDisrupt"), t("TrenchOutpostPoints"), t("OutpostStagingGather"));
      case "Trench Outpost Points" -> List.of(t("TrenchOutpostPoints"), t("OutpostStagingGather"));
      case "Trench Outpost Follow" -> List.of(t("TrenchOutpostFollow"), t("OutpostStagingGather"));
      case "Depot Double Pass", "Depot Double Pass (Shoot First)" -> List.of(
          t("DepotBump"), t("DepotDoublePass"), t("DepotBumpReturn"), m("OutpostStagingGather"));
      case "Depot Single Pass",
          "Depot Single Pass (Shoot First)",
          "Depot Single Pass Shoot On Move" -> List.of(
          t("DepotBump"), t("DepotSinglePass"), t("DepotBumpReturn"), m("OutpostStagingGather"));
      case "Trench Depot Disrupt" -> List.of(
          m("TrenchOutpostDisrupt"), m("TrenchOutpostPoints"), m("OutpostStagingGather"));
      case "Trench Depot Points" -> List.of(m("TrenchOutpostPoints"), m("OutpostStagingGather"));
      case "Trench Depot Follow" -> List.of(m("TrenchOutpostFollow"), m("OutpostStagingGather"));
      default -> List.of();
    };
  }

  private Pose2d[] buildPreviewPoses(String autoName) {
    var poses = new ArrayList<Pose2d>();
    for (TrajEntry e : buildPreviewEntries(autoName)) {
      Choreo.<SwerveSample>loadTrajectory(e.name())
          .ifPresent(
              traj -> {
                var transformed = e.flip() ? traj.mirrorY() : traj;
                if (lastPreviewRed) transformed = transformed.flipped();
                var samples = transformed.samples();
                for (SwerveSample s : samples) {
                  poses.add(s.getPose());
                }
              });
    }
    return poses.toArray(new Pose2d[0]);
  }
}
