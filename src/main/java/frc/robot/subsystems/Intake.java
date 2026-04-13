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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private SparkFlex intakeMotor = new SparkFlex(12, MotorType.kBrushless);
  private final SparkClosedLoopController intakeController;
  private SparkFlex intakeMotor2 = new SparkFlex(11, MotorType.kBrushless);
  private final SparkClosedLoopController intakeController2;
  private double setPoint = 0;
  private static final int intakeCurrentLimit = 60;
  private double gearRatio = 2.8;

  public Intake() {

    // Configure turn motor
    intakeController = intakeMotor.getClosedLoopController();
    var intakeFFConfig = new FeedForwardConfig();
    intakeFFConfig.kV(0.001875);
    var intakeConfig = new SparkFlexConfig();
    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(intakeCurrentLimit)
        .secondaryCurrentLimit(80)
        .voltageCompensation(12.0);
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0003, 0.0, 0.0)
        .apply(intakeFFConfig);
    intakeConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

    SparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Intake Motor 2 configure
    intakeController2 = intakeMotor.getClosedLoopController();
    var intakeConfig2 = new SparkFlexConfig();
    intakeConfig2
        .idleMode(IdleMode.kCoast)
        .follow(12, true)
        .smartCurrentLimit(intakeCurrentLimit)
        .secondaryCurrentLimit(80)
        .voltageCompensation(12.0);
    intakeConfig2
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0003, 0.0, 0.0)
        .apply(intakeFFConfig);
    intakeConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

    SparkUtil.tryUntilOk(
        intakeMotor2,
        5,
        () ->
            intakeMotor2.configure(
                intakeConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @AutoLogOutput(key = "intake/error")
  private double getError() {
    return setPoint - intakeMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "intake/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command reverseIntake() {
    return runOnce(
        () -> {
          setPoint = 1000 * gearRatio;
        });
  }

  public Command startIntake() {
    return runOnce(
        () -> {
          setPoint = -1500 * gearRatio;
        });
  }

  public Command pauseintake() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  @Override
  public void periodic() {

    intakeController.setSetpoint(setPoint, ControlType.kVelocity);

    Logger.recordOutput("intake/Output", intakeMotor.getAppliedOutput());
    Logger.recordOutput("intake/Output2", intakeMotor2.getAppliedOutput());
    Logger.recordOutput("intake/speed", intakeMotor.getEncoder().getVelocity());
    Logger.recordOutput("intake/setPoint", setPoint);
  }
}
