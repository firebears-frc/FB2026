package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ArmSim extends Arm {

  private Rotation2d simAngle = Rotation2d.fromDegrees(0);
  private Rotation2d simSetpoint = Rotation2d.fromDegrees(0);

  private static final double SIM_SPEED_DEG_PER_SEC = 60.0;
  private static final double DT = 0.02;

  public ArmSim() {
    super(); // safe now because we override setShoulderSetpoint
  }

  // Prevent real constructor from crashing
  @Override
  public void setShoulderSetpoint(Rotation2d setpoint) {
    if (setpoint == null) return;
    double deg = setpoint.getDegrees();
    deg = Math.max(-10, Math.min(120, deg));
    simSetpoint = Rotation2d.fromDegrees(deg);
  }

  @Override
  public Command armUp() {
    return runOnce(() -> simSetpoint = Rotation2d.fromDegrees(125));
  }

  @Override
  public Command armDown() {
    return runOnce(() -> simSetpoint = Rotation2d.fromDegrees(-8));
  }

  @Override
  public Command defaultCommand(java.util.function.Supplier<Double> shoulderChange) {
    return run(
        () -> {
          double newDeg = simSetpoint.getDegrees() - shoulderChange.get();
          newDeg = Math.max(-10, Math.min(120, newDeg));
          simSetpoint = Rotation2d.fromDegrees(newDeg);
        });
  }

  @Override
  public Rotation2d getShoulderAngle() {
    return simAngle;
  }

  @Override
  public void periodic() {
    if (RobotBase.isReal()) {
      super.periodic();
      return;
    }

    double current = simAngle.getDegrees();
    double target = simSetpoint.getDegrees();
    double maxStep = SIM_SPEED_DEG_PER_SEC * DT;
    double error = target - current;

    if (Math.abs(error) <= maxStep) {
      simAngle = Rotation2d.fromDegrees(target);
    } else {
      simAngle = Rotation2d.fromDegrees(current + Math.copySign(maxStep, error));
    }

    Logger.recordOutput("arm/angleDegrees", simAngle.getDegrees());
    Logger.recordOutput("arm/setPointDegrees", simSetpoint.getDegrees());
    Logger.recordOutput("arm/simMode", true);
  }
}
