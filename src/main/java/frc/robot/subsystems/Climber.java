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


public class Climber extends SubsystemBase {
  private SparkMax climberMotor = new SparkMax(12, MotorType.kBrushless);
  private final SparkClosedLoopController climberController;
  private double setPoint = 0;
  private static final int climberCurrentLimit = 30;
  private final double climberMaxMotorPosition = 0000.0000;
  private double climberMotorPosition = climberMotor.getEncoder().getPosition();



  public Climber() {

    // Configure turn motor
    climberController = climberMotor.getClosedLoopController();
    var climberConfig = new SparkMaxConfig();
    climberConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(climberCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    climberConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(.00007, 0.0, 0.0, 1.774691358024691e-4);
    // ff: 1.774691358024691e-4
    climberConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    SparkUtil.tryUntilOk(
        climberMotor,
        5,
        () ->
            climberMotor.configure(
                climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @AutoLogOutput(key = "climber/error")
  private double getError() {
    return setPoint - climberMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "climber/atSpeed")
  private boolean atSpeed() {
    return getError() < 100 && getError() > -100;
  }

  public Command climberdown() {
    return runOnce(
        () -> {
          setPoint = -1000;
        });
  }

  public Command climberup() {
    return runOnce(
        () -> {
          setPoint = 1000;
        });
  }

  
  public Command pauseclimber() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }


  @Override
  public void periodic() {
    climberMotorPosition = climberMotor.getEncoder().getPosition();


    if(climberMotorPosition >= climberMaxMotorPosition){
      if(setPoint >= 0){
        setPoint = 0;
      }
    }

    climberController.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("climber/Output", climberMotor.getAppliedOutput());
    Logger.recordOutput("climber/speed", climberMotor.getEncoder().getVelocity());
    Logger.recordOutput("climber/position", climberMotor.getEncoder().getPosition());
    Logger.recordOutput("climber/setPoint", setPoint);
  }
}