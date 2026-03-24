package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.corrections;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSim extends Shooter {

  // Simulated shooter velocity (RPM)
  private double simVelocity = 0.0;

  // Simulated mode ("fast", "slow", "auto", "reverse", "static", "off")
  private String simMode = "off";

  // Simulated setpoint (RPM)
  private double simSetPoint = 0.0;

  // Max speed from real class
  private final double maxSpeed = 5500;

  // Simulated acceleration (RPM per second)
  private static final double SIM_ACCEL_RPM_PER_SEC = 8000.0;
  private static final double DT = 0.02; // 20ms loop

  // Distance supplier (same as real shooter)
  private final DoubleSupplier distanceToHubSupplier;

  // Static shooter speed (same as real shooter)
  private final LoggedNetworkNumber staticShooterSpeed =
      new LoggedNetworkNumber("Static Shooter Speed", 3000);

  // Auto‑mode interpolation table
  private final InterpolatingDoubleTreeMap speedCalculator = new InterpolatingDoubleTreeMap();

  public ShooterSim(DoubleSupplier distanceToHubSupplier) {
    super(distanceToHubSupplier); // runs real constructor, but we override behavior
    this.distanceToHubSupplier = distanceToHubSupplier;

    // Populate interpolation table (same as real shooter)
    speedCalculator.put(2.0, 2900.0);
    speedCalculator.put(2.5, 2900.0);
    speedCalculator.put(2.9, 3100.0);
    speedCalculator.put(3.1, 3250.0);
    speedCalculator.put(3.4, 3350.0);
    speedCalculator.put(3.8, 3400.0);
    speedCalculator.put(4.5, 3800.0);
    speedCalculator.put(5.25, 3950.0);
    speedCalculator.put(6.6, 6500.0);
  }

  // --- Override commands so they update simMode instead of real mode ---

  @Override
  public Command reverseShooter() {
    return runOnce(() -> simMode = "reverse");
  }

  @Override
  public Command autoShooter() {
    return runOnce(() -> simMode = "auto");
  }

  @Override
  public Command fastShot() {
    return runOnce(() -> simMode = "fast");
  }

  @Override
  public Command slowShot() {
    return runOnce(() -> simMode = "slow");
  }

  @Override
  public Command staticShot() {
    return runOnce(() -> simMode = "static");
  }

  @Override
  public Command pauseShooter() {
    return runOnce(() -> simMode = "off");
  }

  @Override
  public Command decreaseStaticSpeed() {
    return runOnce(() -> staticShooterSpeed.set(staticShooterSpeed.get() - 50));
  }

  @Override
  public Command increaseStaticSpeed() {
    return runOnce(() -> staticShooterSpeed.set(staticShooterSpeed.get() + 50));
  }

  @Override
  public String getMode() {
    return simMode;
  }

  // --- Simulation periodic ---

  @Override
  public void periodic() {
    // If running on real hardware, use the real subsystem
    if (RobotBase.isReal()) {
      super.periodic();
      return;
    }
    boolean shooting = simMode.equals("fast") || simMode.equals("slow") || simMode.equals("auto");

    corrections.setDrawShotLine(shooting);

    double distance = distanceToHubSupplier.getAsDouble();

    // Determine simulated setpoint based on mode
    switch (simMode) {
      case "fast":
        simSetPoint = 3500;
        break;
      case "slow":
        simSetPoint = 2600;
        break;
      case "reverse":
        simSetPoint = -2600;
        break;
      case "auto":
        simSetPoint = speedCalculator.get(distance);
        break;
      case "static":
        simSetPoint = staticShooterSpeed.get();
        break;
      default:
        simSetPoint = 0;
        break;
    }

    // Clamp speeds
    if (simSetPoint > maxSpeed) simSetPoint = maxSpeed;
    if (staticShooterSpeed.get() > maxSpeed) staticShooterSpeed.set(maxSpeed);

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
    Logger.recordOutput("Shooter/mode", simMode);
    Logger.recordOutput("Shooter/speed", simVelocity);
    Logger.recordOutput("Shooter/setPoint", simSetPoint);
    Logger.recordOutput("Odometry/distance to hub", distance);
    Logger.recordOutput("ShooterSim/Output", target == 0 ? 0 : Math.signum(target));
  }
}
