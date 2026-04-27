package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.corrections;
import frc.robot.util.SparkUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private static enum HopperState {
    Off,
    Forward,
    Reverse
  }

  private SparkMax hopperMotor = new SparkMax(13, MotorType.kBrushless); // change can id
  private final SparkClosedLoopController hopperController;
  private double setPoint = 0;
  private static final int HopperCurrentLimit = 40; // safety limit
  private HopperState mode = HopperState.Off;
  private double gearRatio = 3;

  public Hopper() {

    // Configure turn motor
    hopperController = hopperMotor.getClosedLoopController();
    var HopperFFConfig = new FeedForwardConfig();
    HopperFFConfig.kV(0.0022);
    var HopperConfig = new SparkMaxConfig();
    HopperConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(HopperCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    HopperConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0003, 0.0, 0.0)
        .apply(HopperFFConfig);
    HopperConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

    SparkUtil.tryUntilOk(
        hopperMotor,
        5,
        () ->
            hopperMotor.configure(
                HopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @AutoLogOutput(key = "Hopper/error")
  private double getError() {
    return setPoint - hopperMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Hopper/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }
  // rip reverse hopper

  // to do what buttons assign speeds
  public Command startHopper() {
    return runOnce(
        () -> {
          mode = HopperState.Forward;
        });
  }

  // to do adept to a button
  public Command pauseHopper() {
    return runOnce(
        () -> {
          mode = HopperState.Off;
        });
  }

  public Command reverseHopper() {
    return runOnce(
        () -> {
          mode = HopperState.Reverse;
        });
  }

  public Command altMode(Supplier<Boolean> shooterIsOn) {
    return runOnce(
        () -> {
          if (shooterIsOn.get()) {
            mode = HopperState.Off;
          } else {
            mode = HopperState.Forward;
          }
        });
  }

  public Command regMode(Supplier<Boolean> shooterIsOn) {
    return runOnce(
        () -> {
          if (shooterIsOn.get()) {
            mode = HopperState.Forward;
          } else {
            mode = HopperState.Off;
          }
        });
  }

  @Override
  public void periodic() {

    if (mode == HopperState.Forward) {
      setPoint = -1400 * gearRatio;
    } else if (mode == HopperState.Reverse) {
      setPoint = 700 * gearRatio;
    } else {
      setPoint = 0;
    }
    // turns off the hopper if behind the opposing hub so we don't score for our opponents
    if (corrections.currentZone() == 2 && corrections.behindHub()) {
      setPoint = 0;
    }
    hopperController.setSetpoint(setPoint, ControlType.kVelocity);

    Logger.recordOutput("Hopper/mode", mode);
    Logger.recordOutput("Hopper/Output", hopperMotor.getAppliedOutput());
    Logger.recordOutput("Hopper/speed", hopperMotor.getEncoder().getVelocity());
    Logger.recordOutput("Hopper/setPoint", setPoint);
  }
}
