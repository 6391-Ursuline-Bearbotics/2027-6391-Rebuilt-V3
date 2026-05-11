package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  // CAN IDs
  public static final int leftMotorId = 7;
  public static final int rightMotorId = 8;
  public static final CANBus canBus = new CANBus("");

  // Motor configuration
  public static final double gearRatio = 1.0;
  public static final boolean leftMotorInverted = true;
  public static final boolean rightMotorInverted = false; // Mirrors left
  public static final double statorCurrentLimit = 80.0;
  public static final Slot0Configs gains = new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Velocity setpoints
  public static final double idleRPM = 0.0;
  public static final double ejectRPM = -2000.0;

  // Setpoint tolerance for isAtSetpoint()
  public static final double toleranceRPM = 100.0;
  // Passing shots don't need precise speed — wider tolerance so the gate opens under low battery
  public static final double passToleranceRPM = 1500.0;

  // Bang-bang FOC mode
  public static final double bangBangKp = 999999.0;
  public static final double bangBangPeakCurrentAmps = 80.0;

  // Shoot-on-the-move
  public static final int shotCompensationIterations = 2;

  /**
   * Physical heading offset of the shooter relative to the robot's back axis, in degrees. Positive
   * values rotate the aim clockwise (when viewed from above). Tune this if shots consistently miss
   * to one side.
   */
  public static final double shooterHeadingOffsetDegrees = 4.0;

  // Hood servos (PWM ports - placeholder, update when wired)
  public static final int hoodLeftServoPWM = 0;
  public static final int hoodRightServoPWM = 1;
  public static final double hoodMinAngleDeg = 20.0;
  public static final double hoodMaxAngleDeg = 45.0;

  // Jam detection
  public static final double jamCurrentThreshold = 60.0; // Stator amps
  public static final double jamDebounceTime = 1.0; // Seconds

  // Trench approach — hood must be lowered before entering a trench
  public static final double trenchApproachDistanceMeters = 2.0;
  public static final double trenchApproachMinVelocityMps = 0.3;
  public static final double trenchApproachXMarginMeters = 0.5;

  // Simulation
  public static final double simMOI = 0.001;

  /** Distance (meters) -> Shooter RPM for hub shots (alliance area). */
  public static InterpolatingDoubleTreeMap createHubDistanceToRPMMap() {
    var map = new InterpolatingDoubleTreeMap();
    map.put(1.8, 2500.0); // auto starting point
    map.put(2.0, 2600.0);
    map.put(2.5, 2800.0);
    map.put(3.0, 3000.0); // trench shot
    map.put(3.5, 3200.0);
    map.put(4.0, 3250.0);
    map.put(4.5, 3450.0);
    map.put(5.1, 3450.0);
    map.put(5.15, 3450.0);
    return map;
  }

  /** Distance (meters) -> Hood angle (degrees) for hub shots (alliance area). */
  public static InterpolatingDoubleTreeMap createHubDistanceToAngleMap() {
    var map = new InterpolatingDoubleTreeMap();
    map.put(1.5, 26.0); // 26 is our lowest angle
    map.put(1.8, 26.0);
    map.put(2.5, 26.0);
    map.put(3.0, 27.5);
    map.put(3.5, 28.5);
    map.put(4.0, 29.5);
    map.put(4.5, 31.0);
    map.put(5.0, 32.5);
    map.put(5.25, 32.0);
    map.put(5.8, 38.0);
    return map;
  }

  /** Distance (meters) -> Shooter RPM for passing shots. */
  public static InterpolatingDoubleTreeMap createPassDistanceToRPMMap() {
    var map = new InterpolatingDoubleTreeMap();
    // TODO: Populate with measured passing data
    map.put(3.6, 4200.0);
    map.put(5.0, 5350.0);
    map.put(6.0, 5600.0);
    map.put(6.5, 6000.0);
    return map;
  }

  /** Distance (meters) -> Hood angle (degrees) for passing shots. */
  public static InterpolatingDoubleTreeMap createPassDistanceToAngleMap() {
    var map = new InterpolatingDoubleTreeMap();
    // TODO: Populate with measured passing data
    map.put(5.0, 40.0);
    map.put(6.0, 40.0);
    map.put(6.5, 40.0);
    return map;
  }

  /**
   * Distance (meters) -> Time of flight (seconds) interpolation table for shoot-on-the-move
   * compensation. Populate with empirical data from testing. Values below are placeholders.
   */
  public static InterpolatingDoubleTreeMap createDistanceToTOFMap() {
    var map = new InterpolatingDoubleTreeMap();
    // TODO: Replace with real measured data
    map.put(2.0, 0.85);
    map.put(3.0, 0.90);
    map.put(4.0, 0.95);
    map.put(5.0, 1.0);
    map.put(6.0, 1.05);
    map.put(7.0, 1.10);
    return map;
  }
}
