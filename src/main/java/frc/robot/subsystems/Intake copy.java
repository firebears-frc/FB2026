package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor = new SparkMax(12, MotorType.kBrushless);
  private final SparkClosedLoopController intakeController;
  private SparkLimitSwitch beamBreak = intakeMotor.getForwardLimitSwitch();
  private double setPoint = 0;
  private static final int intakeCurrentLimit = 30;

  @AutoLogOutput(key = "intake/hasCoral")
  private boolean hasCoral = false;

  public Intake() {

    // Configure turn motor
    intakeController = intakeMotor.getClosedLoopController();
    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(intakeCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(.00007, 0.0, 0.0, 1.774691358024691e-4);
    // ff: 1.774691358024691e-4
    intakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    SparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @AutoLogOutput(key = "intake/beamBreak")
  private boolean beamBreak() {
    return beamBreak.isPressed();
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
          setPoint = -1000;
        });
  }

  public Command startIntake() {
    return runOnce(
        () -> {
          setPoint = 1000;
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
    if (beamBreak() && !hasCoral) {
      setPoint = 0;
      hasCoral = true;
    } else if (!beamBreak()) {
      hasCoral = false;
    }

    intakeController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("intake/Output", intakeMotor.getAppliedOutput());
    Logger.recordOutput("intake/speed", intakeMotor.getEncoder().getVelocity());
    Logger.recordOutput("intake/setPoint", setPoint);
  }
}