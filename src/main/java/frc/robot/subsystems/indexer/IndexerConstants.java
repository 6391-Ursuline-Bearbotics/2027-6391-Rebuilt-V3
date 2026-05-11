package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

public final class IndexerConstants {
  // CAN IDs
  public static final int beltMotorId = 5;
  public static final int kickerMotorId = 6;
  public static final CANBus canBus = new CANBus("");

  // Belt motor
  public static final double beltGearRatio = 1.0;
  public static final boolean beltMotorInverted = true;
  public static final double beltStatorCurrentLimit = 80.0;
  public static final double beltFeedVelocityRPM = 900.0;
  public static final double beltEjectVelocityRPM = -1000.0;
  public static final Slot0Configs beltGains =
      new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Kicker motor
  public static final double kickerGearRatio = 1.0;
  public static final boolean kickerMotorInverted = true;
  public static final double kickerStatorCurrentLimit = 80.0;
  public static final double kickerFeedVelocityRPM = 2900.0;
  public static final double kickerEjectVelocityRPM = -2900.0;
  public static final Slot0Configs kickerGains =
      new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Jam detection
  public static final double jamCurrentThreshold = 40.0; // Stator amps
  public static final double jamDebounceTime = 0.5; // Seconds
  public static final double jamReverseTime = 0.25; // Seconds to reverse

  // Spinner motors (NEO 550 + SparkMAX)
  public static final int leftSpinnerMotorId = 9;
  public static final int rightSpinnerMotorId = 10;
  public static final double spinnerGearRatio = 12.0 / 60.0; // 12:60 reduction
  public static final double spinnerSpeed = 0.5; // duty cycle, -1.0 to 1.0
  public static final double spinnerDelaySeconds = 0.5; // delay after kicker/belt start
  public static final double spinnerStallCurrentAmps = 15.0; // stall detection threshold
  public static final double spinnerCooldownSeconds = 1.0; // off-time after stall
  public static final int spinnerCurrentLimitAmps = 20; // sustained smart current limit
  public static final int spinnerInrushCurrentAmps = 40; // brief startup inrush allowance
  public static final double spinnerInrushTimeSeconds = 0.35; // how long inrush is permitted

  // Simulation
  public static final double beltSimMOI = 0.001;
  public static final double kickerSimMOI = 0.001;
}
