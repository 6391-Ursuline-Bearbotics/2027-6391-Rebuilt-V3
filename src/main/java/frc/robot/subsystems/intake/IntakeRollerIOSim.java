package frc.robot.subsystems.intake;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private static final DCMotor GEARBOX = DCMotor.getFalcon500(1);
  private static final double DEFAULT_SIM_KP = 0.05;
  private static final double DEFAULT_SIM_KV = 0.12;
  private static final double DEFAULT_SIM_KS = 0.0;

  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(DEFAULT_SIM_KP, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double simKv = DEFAULT_SIM_KV;
  private double simKs = DEFAULT_SIM_KS;

  public IntakeRollerIOSim() {
    sim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                GEARBOX, IntakeConstants.rollerSimMOI, IntakeConstants.rollerGearRatio),
            GEARBOX);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = ffVolts + controller.calculate(sim.getAngularVelocity());
    } else {
      controller.reset();
    }

    sim.setInputVoltage(Math.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDraw());
    inputs.supplyCurrentAmps = inputs.statorCurrentAmps * Math.abs(appliedVolts) / 12.0;
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    ffVolts = simKs * Math.signum(velocityRadPerSec) + simKv * velocityRadPerSec;
    controller.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void setGains(double kP, double kV, double kS) {
    controller.setP(kP);
    simKv = kV;
    simKs = kS;
  }
}
