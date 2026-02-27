package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private SparkFlex ShooterMotor1 = new SparkFlex(14, MotorType.kBrushless);
  private SparkFlex ShooterMotor2 = new SparkFlex(15, MotorType.kBrushless);
  private final SparkClosedLoopController ShooterController1;
  private final SparkClosedLoopController ShooterController2;
  private SparkLimitSwitch beamBreak = ShooterMotor1.getForwardLimitSwitch();
  private double setPoint = 0;
  // Variables that can be updated
  private static final int smartShooterCurrentLimit = 75;
  private static final int secondaryShooterCurrentLimit = 85;
  private final double motorP = 0.0001;
  private final double motorI = 0.0;
  private final double motorD = 0.0;
  private final double motorFF = 0.00185;
  private String mode = "";
  InterpolatingDoubleTreeMap speedCalculator = new InterpolatingDoubleTreeMap();

  @AutoLogOutput(key = "Shooter/fuel ready")
  private boolean FuelReady = false;

  private final DoubleSupplier distanceToHubSupplier;

  public Shooter(DoubleSupplier distanceToHubSupplier) {
    this.distanceToHubSupplier = distanceToHubSupplier;
    // Configure Motor 1
    ShooterController1 = ShooterMotor1.getClosedLoopController();
    var ShooterConfig1 = new SparkFlexConfig();
    ShooterConfig1.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(smartShooterCurrentLimit)
        .secondaryCurrentLimit(secondaryShooterCurrentLimit)
        .voltageCompensation(12.0);
    ShooterConfig1.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(motorP, motorI, motorD, motorFF);
    ShooterConfig1.limitSwitch.forwardLimitSwitchEnabled(false);

    SparkUtil.tryUntilOk(
        ShooterMotor1,
        5,
        () ->
            ShooterMotor1.configure(
                ShooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure Motor 2
    ShooterController2 = ShooterMotor2.getClosedLoopController();
    var ShooterConfig2 = new SparkFlexConfig();
    ShooterConfig2.idleMode(IdleMode.kCoast)
        .follow(14, true)
        .smartCurrentLimit(smartShooterCurrentLimit)
        .secondaryCurrentLimit(secondaryShooterCurrentLimit)
        .voltageCompensation(12.0);
    ShooterConfig2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    ShooterConfig2.limitSwitch.forwardLimitSwitchEnabled(false);

    SparkUtil.tryUntilOk(
        ShooterMotor2,
        5,
        () ->
            ShooterMotor2.configure(
                ShooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Populate speed calculator with values (subject to change based on testing)
    speedCalculator.put(Units.feetToMeters(15), 3500.0);
    speedCalculator.put(Units.feetToMeters(12), 3000.0);
    speedCalculator.put(Units.feetToMeters(8), 2700.0);
    speedCalculator.put(Units.inchesToMeters(80), 2600.0);
  }

  @AutoLogOutput(key = "Shooter/beamBreak")
  private boolean beamBreak() {
    return beamBreak.isPressed();
  }

  @AutoLogOutput(key = "Shooter/error")
  private double getError() {
    return setPoint - (ShooterMotor1.getEncoder().getVelocity());
  }

  @AutoLogOutput(key = "Shooter/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command reverseShooter() {
    return runOnce(
        () -> {
          mode = "reverse";
        });
  }

  public Command autoShooter() {
    return runOnce(
        () -> {
          mode = "auto";
        });
  }

  public Command fastShot() {
    return runOnce(
      () -> {
        mode = "fast";
      });
  }
  // subject to change based on design of the motor and mechanism
  public Command slowShot() {
    return runOnce(
        () -> {
          mode = "slow";
        });
  }

  public Command pauseShooter() {
    return runOnce(
        () -> {
          mode = "off";
        });
  }

  @Override
  public void periodic() {
    if (beamBreak() && !FuelReady) {
      //  setPoint = 0;
      FuelReady = true;
    } else if (!beamBreak()) {
      FuelReady = false;
    }

    if (mode == "fast") {
      setPoint = 3500;
    } else if (mode == "slow") {
      setPoint = 2600;
    } else if (mode == "reverse") {
      setPoint = -2600;
    } else if (mode == "auto") {
      setPoint = speedCalculator.get(distanceToHubSupplier.getAsDouble());
    } else {
      setPoint = 0;
    }

    ShooterController1.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("Shooter1/Output", ShooterMotor1.getAppliedOutput());
    Logger.recordOutput("Shooter2/Output", ShooterMotor2.getAppliedOutput());
    Logger.recordOutput("Shooter/speed", ShooterMotor1.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/setPoint", setPoint);
  }
}
