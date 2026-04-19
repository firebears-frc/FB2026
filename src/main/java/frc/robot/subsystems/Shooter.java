package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.corrections;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  private static enum ShooterState {
    Off,
    Static,
    Auto,
    Sotm,
    Reverse,
    Fast,
    Slow
  }

  private SparkFlex ShooterMotor1 = new SparkFlex(14, MotorType.kBrushless);
  private SparkFlex ShooterMotor2 = new SparkFlex(15, MotorType.kBrushless);
  private final SparkClosedLoopController ShooterController1;
  private final SparkClosedLoopController ShooterController2;

  // private final LaserCan lc;//It is laser coming through the shooter
  private double setPoint = 0;

  // Variables that can be updated
  private static final int smartShooterCurrentLimit = 75;
  private static final int secondaryShooterCurrentLimit = 85;
  private final double motorP = 0.0003; // 0.000175
  private final double motorI = 0.0;
  private final double motorD = 0.0;
  private final double motorFF = 0.0018; // 0.0018
  private final double maxSpeed = 6500;
  InterpolatingDoubleTreeMap speedCalculator = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap relayCalculator = new InterpolatingDoubleTreeMap();
  private ShooterState mode = ShooterState.Off;

  // Dashboard Input
  private LoggedNetworkNumber staticShooterSpeed =
      new LoggedNetworkNumber("Static Shooter Speed", 3000);
  private LoggedNetworkNumber ShootAdjustment = new LoggedNetworkNumber("Shoot Adjustment", 1.00);
  private LoggedNetworkNumber shooterAngleOffset = new LoggedNetworkNumber("angleOffset", 0);

  private final DoubleSupplier distanceToHubSupplier;

  public Shooter(DoubleSupplier distanceToHubSupplier) {
    this.distanceToHubSupplier = distanceToHubSupplier;

    // //laser
    //  lc = new LaserCan(16);

    // try {
    // lc.setRangingMode(LaserCan.RangingMode.SHORT);
    //  lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //  lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    // } catch (ConfigurationFailedException e) {
    // System.out.println("Configuration failed! " + e);
    // }

    // Configure Motor 1
    ShooterController1 = ShooterMotor1.getClosedLoopController();
    var ShooterFFConfig1 = new FeedForwardConfig();
    ShooterFFConfig1.kV(motorFF);
    var ShooterConfig1 = new SparkFlexConfig();
    ShooterConfig1.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(smartShooterCurrentLimit)
        .secondaryCurrentLimit(secondaryShooterCurrentLimit)
        .voltageCompensation(12.0);
    ShooterConfig1.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(motorP, motorI, motorD)
        .apply(ShooterFFConfig1);
    ShooterConfig1.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    ShooterConfig1.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(4);

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
    ShooterConfig2.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    ShooterConfig2.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(4);

    SparkUtil.tryUntilOk(
        ShooterMotor2,
        5,
        () ->
            ShooterMotor2.configure(
                ShooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Populate speed calculator with values (subject to change based on testing) (Meters from hub | RPM)
    // updated on 4/18/26
    speedCalculator.put(2.2, 2850.0);
    speedCalculator.put(2.5, 2875.0);
    speedCalculator.put(3.0, 2925.0);
    speedCalculator.put(3.5, 3100.0);
    speedCalculator.put(4.0, 3300.0);
    speedCalculator.put(5.0, 3725.0);
    speedCalculator.put(6.0, 4400.0);

    // Populate relay calculator with values (Meters from bump/trench line | RPM)
    // ~unchanged values, a couple added~
    relayCalculator.put(2.2, 2850.0);
    relayCalculator.put(2.5, 2875.0);
    relayCalculator.put(3.0, 2925.0);
    relayCalculator.put(3.5, 3100.0);
    relayCalculator.put(4.0, 3300.0);
    relayCalculator.put(5.0, 3725.0);
    relayCalculator.put(6.0, 4400.0);
    relayCalculator.put(7.0, 5000.0);
    relayCalculator.put(8.0, 6000.0);
    relayCalculator.put(9.0, 6500.0);
  }

  @AutoLogOutput(key = "Shooter/error")
  private double getError() {
    return setPoint - (ShooterMotor1.getEncoder().getVelocity());
  }

  @AutoLogOutput(key = "Shooter/atSpeed")
  public boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command reverseShooter() {
    return runOnce(
        () -> {
          mode = ShooterState.Reverse;
          corrections.setDrawShotLine(false);
        });
  }

  public Command autoShooter() {
    return runOnce(
        () -> {
          mode = ShooterState.Auto;
          corrections.setDrawShotLine(true);
        });
  }

  public Command sotmAutoShooter() {
    return runOnce(
        () -> {
          mode = ShooterState.Sotm;
          corrections.setDrawShotLine(true);
        });
  }

  public Command fastShot() {
    return runOnce(
        () -> {
          mode = ShooterState.Fast;
          corrections.setDrawShotLine(true);
        });
  }
  // subject to change based on design of the motor and mechanism
  public Command slowShot() {
    return runOnce(
        () -> {
          mode = ShooterState.Slow;
          corrections.setDrawShotLine(true);
        });
  }

  public Command staticShot() {
    return runOnce(
        () -> {
          mode = ShooterState.Static;
          corrections.setDrawShotLine(true);
        });
  }

  public Boolean isRunning() {
    return mode != ShooterState.Off;
  }

  public Command decreaseStaticSpeed() {
    return runOnce(
            () -> {
              staticShooterSpeed.set(staticShooterSpeed.get() - 25);
            })
        .ignoringDisable(true);
  }

  public Command increaseStaticSpeed() {
    return runOnce(
            () -> {
              staticShooterSpeed.set(staticShooterSpeed.get() + 25);
            })
        .ignoringDisable(true);
  }

  public Command decreaseShootAdjustment() {
    return runOnce(
            () -> {
              ShootAdjustment.set(ShootAdjustment.get() - 0.005);
            })
        .ignoringDisable(true);
  }

  public Command increaseShootAdjustment() {
    return runOnce(
            () -> {
              ShootAdjustment.set(ShootAdjustment.get() + 0.005);
            })
        .ignoringDisable(true);
  }

  // decreases the angle where the rbot shoots the ball untill 75
  public Command decreaseAngleAdjustment() {
    return runOnce(
            () -> {
              shooterAngleOffset.set(Math.max(shooterAngleOffset.get() - 0.5, -15));
              corrections.setShooterAngleOffset(Math.toRadians(shooterAngleOffset.get()));
            })
        .ignoringDisable(true);
  }

  // increases the angle where the rbot shoots the ball untill 105
  public Command increaseAngleAdjustment() {
    return runOnce(
            () -> {
              shooterAngleOffset.set(Math.min(shooterAngleOffset.get() + 0.5, 15));
              corrections.setShooterAngleOffset(Math.toRadians(shooterAngleOffset.get()));
            })
        .ignoringDisable(true);
  }

  public Command pauseShooter() {
    return runOnce(
        () -> {
          mode = ShooterState.Off;
          corrections.setDrawShotLine(false);
        });
  }

  @Override
  public void periodic() {

    // Get the distance from the hub
    double distance = distanceToHubSupplier.getAsDouble();
    if (ShootAdjustment.get() > 1.05) {
      ShootAdjustment.set(1.05);
    }

    if (ShootAdjustment.get() < .95) {
      ShootAdjustment.set(.95);
    }

    if (mode == ShooterState.Fast) {
      setPoint = 3500;
    } else if (mode == ShooterState.Slow) {
      setPoint = 2600;
    } else if (mode == ShooterState.Reverse) {
      setPoint = -2600;
    } else if (mode == ShooterState.Auto) {
      setPoint = speedCalculator.get(distance) * ShootAdjustment.get();
    } else if (mode == ShooterState.Sotm) {
      if (corrections.currentZone() <= 0){
        setPoint = speedCalculator.get(corrections.sotmGetDistance()) * ShootAdjustment.get();
      } else {
        setPoint = relayCalculator.get(corrections.distanceToSide());
      }
      
    } else if (mode == ShooterState.Static) {
      setPoint = staticShooterSpeed.get();
    } else {
      setPoint = 0;
    }

    if (staticShooterSpeed.get() > maxSpeed) {
      staticShooterSpeed.set(maxSpeed);
    }

    if (setPoint > maxSpeed) {
      setPoint = maxSpeed;
    }

    ShooterController1.setSetpoint(setPoint, ControlType.kVelocity);

    // LaserCan.Measurement measurement = lc.getMeasurement();
    // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
    // {
    //   Logger.recordOutput("Shooter/laser", measurement.distance_mm);
    //    Logger.recordOutput("Shooter/status", "reliable");
    // } else {
    //   Logger.recordOutput("Shooter/laser", measurement.distance_mm);
    //    Logger.recordOutput("Shooter/status","unreliable");
    //   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
    // unreliable measurement.
    // }

    Logger.recordOutput("Shooter1/Output", ShooterMotor1.getAppliedOutput());
    Logger.recordOutput("Shooter2/Output", ShooterMotor2.getAppliedOutput());
    Logger.recordOutput("Shooter/mode", mode);
    Logger.recordOutput("Shooter/speed", ShooterMotor1.getEncoder().getVelocity());
    Logger.recordOutput("Odometry/distance to hub", distance);
    Logger.recordOutput("Shooter/setPoint", setPoint);
  }
}
