package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

public final class IntakeConstants {
  // CAN IDs (placeholder - update to match robot wiring)
  public static final int deployMotorId = 4;
  public static final int rollerMotorId = 3;
  public static final CANBus canBus = new CANBus("");

  // Deploy motor
  public static final double deployGearRatio = 25.0;
  public static final boolean deployMotorInverted = false;
  public static final double deployVoltage = 2.8;
  public static final double retractVoltage = -2.5;
  public static final double deployCurrentThreshold = 30.0; // Stator amps indicating hard stop
  public static final double retractCurrentThreshold =
      55.0; // Stator amps indicating retracted hard stop (20% more than deploy)
  public static final double deployInrushIgnoreTime = 0.15; // Seconds to ignore after motor starts
  public static final double deployStatorCurrentLimit = 60.0;

  // Roller motor
  public static final double rollerGearRatio = 1.0;
  public static final boolean rollerMotorInverted = true;
  public static final double rollerIntakeVelocityRPM = 3600.0;
  public static final double rollerClumpIntakeVelocityRPM = 4000.0;
  public static final double rollerEjectVelocityRPM = -3600.0;
  public static final double rollerStatorCurrentLimit = 90.0;
  public static final Slot0Configs rollerGains =
      new Slot0Configs().withKP(0.6).withKV(0.119).withKS(0.24691);

  // Roller jam detection
  public static final double rollerJamCurrentThreshold = 40.0; // Stator amps
  public static final double rollerJamDebounceTime = 0.5; // Seconds
  public static final double rollerJamReverseTime = 0.25; // Seconds to reverse

  // Simulation
  public static final double deploySimMOI = 0.01;
  public static final double rollerSimMOI = 0.001;
}
