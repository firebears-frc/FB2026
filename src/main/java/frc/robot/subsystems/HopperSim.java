package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class HopperSim extends Hopper {

  // Simulated hopper velocity (RPM)
  private double simVelocity = 0.0;

  // Simulated mode ("forward", "reverse", "off")
  private String simMode = "off";

  // Simulated setpoint (RPM)
  private double simSetPoint = 0.0;

  // How fast the simulated hopper changes speed (RPM per second)
  private static final double SIM_ACCEL_RPM_PER_SEC = 5000.0;
  private static final double DT = 0.02; // 20ms loop

  public HopperSim() {
    super(); // runs your constructor, but we override behavior
  }

  // --- Override commands so they update simMode instead of the real mode ---

  @Override
  public Command startHopper() {
    return runOnce(() -> simMode = "forward");
  }

  @Override
  public Command pauseHopper() {
    return runOnce(() -> simMode = "off");
  }

  @Override
  public Command reverseHopper() {
    return runOnce(() -> simMode = "reverse");
  }

  @Override
  public Command altMode(java.util.function.Supplier<String> shooterMode) {
    return runOnce(
        () -> {
          if (shooterMode.get().equals("off")) {
            simMode = "forward";
          } else {
            simMode = "off";
          }
        });
  }

  @Override
  public Command regMode(java.util.function.Supplier<String> shooterMode) {
    return runOnce(
        () -> {
          if (shooterMode.get().equals("off")) {
            simMode = "off";
          } else {
            simMode = "forward";
          }
        });
  }

  // --- Simulation periodic ---

  @Override
  public void periodic() {
    // If running on real hardware, use the real subsystem
    if (RobotBase.isReal()) {
      super.periodic();
      return;
    }

    // Determine simulated setpoint based on mode
    switch (simMode) {
      case "forward":
        simSetPoint = -5250;
        break;
      case "reverse":
        simSetPoint = 1800;
        break;
      default:
        simSetPoint = 0;
        break;
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
    Logger.recordOutput("Hopper/mode", simMode);
    Logger.recordOutput("Hopper/Output", target == 0 ? 0 : Math.signum(target));
    Logger.recordOutput("Hopper/speed", simVelocity);
    Logger.recordOutput("Hopper/setPoint", target);
  }
}
