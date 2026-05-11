package frc.robot.subsystems.indexer;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.plant.DCMotor;
import org.wpilib.math.system.plant.LinearSystemId;
import org.wpilib.simulation.DCMotorSim;

public class IndexerBeltIOSim implements IndexerBeltIO {
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

  public IndexerBeltIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                GEARBOX, IndexerConstants.beltSimMOI, IndexerConstants.beltGearRatio),
            GEARBOX);
  }

  @Override
  public void updateInputs(IndexerBeltIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = ffVolts + controller.calculate(sim.getAngularVelocityRadPerSec());
    } else {
      controller.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
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
