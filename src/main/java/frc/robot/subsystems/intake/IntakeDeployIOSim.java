package frc.robot.subsystems.intake;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

public class IntakeDeployIOSim implements IntakeDeployIO {
  private static final DCMotor GEARBOX = DCMotor.getFalcon500(1);

  // Hard stop boundaries in radians at the output shaft
  private static final double MIN_POSITION_RAD = 0.0;
  private static final double MAX_POSITION_RAD = Math.PI / 2.0; // ~90 degrees of travel

  private final DCMotorSim sim;
  private double appliedVolts = 0.0;

  public IntakeDeployIOSim() {
    sim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                GEARBOX, IntakeConstants.deploySimMOI, IntakeConstants.deployGearRatio),
            GEARBOX);
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    sim.setInputVoltage(Math.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    double position = sim.getAngularPosition();
    boolean atHardStop = false;

    // Model hard stops by clamping position and detecting contact
    if (position <= MIN_POSITION_RAD && appliedVolts < 0.0) {
      atHardStop = true;
      sim.setState(MIN_POSITION_RAD, 0.0);
    } else if (position >= MAX_POSITION_RAD && appliedVolts > 0.0) {
      atHardStop = true;
      sim.setState(MAX_POSITION_RAD, 0.0);
    }

    inputs.connected = true;
    inputs.positionRad = sim.getAngularPosition();
    inputs.velocityRadPerSec = sim.getAngularVelocity();
    inputs.appliedVolts = appliedVolts;
    // Report elevated current when hitting hard stop to trigger stall detection
    inputs.statorCurrentAmps =
        atHardStop
            ? IntakeConstants.deployCurrentThreshold + 20.0
            : Math.abs(sim.getCurrentDraw());
    inputs.supplyCurrentAmps = inputs.statorCurrentAmps * Math.abs(appliedVolts) / 12.0;
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }

  @Override
  public void setBrakeMode(boolean enable) {}
}
