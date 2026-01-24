package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private SparkMax ShooterMotor = new SparkMax(12, MotorType.kBrushless);
  private final SparkClosedLoopController ShooterController;
  private SparkLimitSwitch beamBreak = ShooterMotor.getForwardLimitSwitch();
  private double setPoint = 0;
  private static final int ShooterCurrentLimit = 30;

  @AutoLogOutput(key = "Shooter/fuel ready")
  private boolean FuelReady = false;

  public Shooter() {

    // Configure turn motor
    ShooterController = ShooterMotor.getClosedLoopController();
    var ShooterConfig = new SparkMaxConfig();
    ShooterConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    ShooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(.00007, 0.0, 0.0, 1.774691358024691e-4);
    // ff: 1.774691358024691e-4
    ShooterConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    SparkUtil.tryUntilOk(
        ShooterMotor,
        5,
        () ->
            ShooterMotor.configure(
                ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @AutoLogOutput(key = "Shooter/beamBreak")
  private boolean beamBreak() {
    return beamBreak.isPressed();
  }

  @AutoLogOutput(key = "Shooter/error")
  private double getError() {
    return setPoint - ShooterMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Shooter/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command reverseShooter() {
    return runOnce(
        () -> {
          setPoint = -1000;
        });
  }

  public Command startShooter() {
    return runOnce(
        () -> {
          setPoint = 1000;
        });
  }
  // subject to change based on design of the motor and mechanism
  public Command SlowShot() {
    return runOnce(
        () -> {
          setPoint = 500;
        });
  }

  public Command pauseShooter() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  @Override
  public void periodic() {
    if (beamBreak() && !FuelReady) {
      setPoint = 0;
      FuelReady = true;
    } else if (!beamBreak()) {
      FuelReady = false;
    }

    ShooterController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("Shooter/Output", ShooterMotor.getAppliedOutput());
    Logger.recordOutput("Shooter/speed", ShooterMotor.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/setPoint", setPoint);
  }
}
