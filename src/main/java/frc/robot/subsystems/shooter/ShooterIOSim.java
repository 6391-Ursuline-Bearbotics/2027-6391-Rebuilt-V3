package frc.robot.subsystems.shooter;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  private static final double DEFAULT_SIM_KP = 0.05;
  private static final double DEFAULT_SIM_KV = 0.12;
  private static final double DEFAULT_SIM_KS = 0.0;

  private final DCMotorSim leftSim;
  private final DCMotorSim rightSim;
  private final PIDController leftController = new PIDController(DEFAULT_SIM_KP, 0, 0);
  private final PIDController rightController = new PIDController(DEFAULT_SIM_KP, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private double simKv = DEFAULT_SIM_KV;
  private double simKs = DEFAULT_SIM_KS;

  public ShooterIOSim() {
    leftSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                GEARBOX, ShooterConstants.simMOI, ShooterConstants.gearRatio),
            GEARBOX);
    rightSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                GEARBOX, ShooterConstants.simMOI, ShooterConstants.gearRatio),
            GEARBOX);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts = ffVolts + leftController.calculate(leftSim.getAngularVelocity());
      rightAppliedVolts =
          ffVolts + rightController.calculate(rightSim.getAngularVelocity());
    } else {
      leftController.reset();
      rightController.reset();
    }

    leftSim.setInputVoltage(Math.clamp(leftAppliedVolts, -12.0, 12.0));
    rightSim.setInputVoltage(Math.clamp(rightAppliedVolts, -12.0, 12.0));
    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.leftConnected = true;
    inputs.leftVelocityRadPerSec = leftSim.getAngularVelocity();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftStatorCurrentAmps = Math.abs(leftSim.getCurrentDraw());
    inputs.leftSupplyCurrentAmps = inputs.leftStatorCurrentAmps * Math.abs(leftAppliedVolts) / 12.0;
    inputs.leftTempCelsius = 25.0;

    inputs.rightConnected = true;
    inputs.rightVelocityRadPerSec = rightSim.getAngularVelocity();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightStatorCurrentAmps = Math.abs(rightSim.getCurrentDraw());
    inputs.rightSupplyCurrentAmps =
        inputs.rightStatorCurrentAmps * Math.abs(rightAppliedVolts) / 12.0;
    inputs.rightTempCelsius = 25.0;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    ffVolts = simKs * Math.signum(velocityRadPerSec) + simKv * velocityRadPerSec;
    leftController.setSetpoint(velocityRadPerSec);
    rightController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    leftAppliedVolts = volts;
    rightAppliedVolts = volts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    leftAppliedVolts = 0.0;
    rightAppliedVolts = 0.0;
  }

  @Override
  public void setGains(double kP, double kV, double kS) {
    leftController.setP(kP);
    rightController.setP(kP);
    simKv = kV;
    simKs = kS;
  }
}
