package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class IntakeSim extends Intake {

  // Simulated velocity (RPM)
  private double simVelocity = 0.0;

  // Simulated setpoint (RPM)
  private double simSetPoint = 0.0;

  // How fast the simulated intake changes speed (RPM per second)
  private static final double SIM_ACCEL_RPM_PER_SEC = 4000.0;
  private static final double DT = 0.02; // 20ms loop

  public IntakeSim() {
    super(); // runs your constructor, but we override hardware behavior
  }

  // --- Override commands so they update simSetPoint instead of the real motor ---

  @Override
  public Command startIntake() {
    return runOnce(() -> simSetPoint = -2000);
  }

  @Override
  public Command reverseIntake() {
    return runOnce(() -> simSetPoint = 1000);
  }

  @Override
  public Command pauseintake() {
    return runOnce(() -> simSetPoint = 0);
  }

  // --- Simulation periodic ---

  @Override
  public void periodic() {
    // If running on real hardware, use the real subsystem
    if (RobotBase.isReal()) {
      super.periodic();
      return;
    }

    // Move simulated velocity toward simSetPoint
    double current = simVelocity;
    double target = simSetPoint;

    double maxStep = SIM_ACCEL_RPM_PER_SEC * DT;
    double error = target - current;

    if (Math.abs(error) <= maxStep) {
      simVelocity = target;
    } else {
      simVelocity = current + Math.copySign(maxStep, error);
    }

    // Log simulated values
    Logger.recordOutput("intake/Output", target == 0 ? 0 : Math.signum(target));
    Logger.recordOutput("intake/speed", simVelocity);
    Logger.recordOutput("intake/setPoint", target);
  }
}
