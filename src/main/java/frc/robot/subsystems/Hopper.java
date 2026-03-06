package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private SparkMax hopperMotor = new SparkMax(13, MotorType.kBrushless); // change can id
  private final SparkClosedLoopController hopperController;
  private double setPoint = 0;
  private static final int HopperCurrentLimit = 40; // safety limit
  private String mode = "off";

  public Hopper() {

    // Configure turn motor
    hopperController = hopperMotor.getClosedLoopController();
    var HopperConfig = new SparkMaxConfig();
    HopperConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(HopperCurrentLimit)
        .secondaryCurrentLimit(50)
        .voltageCompensation(12.0);
    HopperConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0.000175, 0.0, 0.0, 0.0022);
    HopperConfig.limitSwitch.forwardLimitSwitchEnabled(false);

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
          mode = "forward";
        });
  }

  // to do adept to a button
  public Command pauseHopper() {
    return runOnce(
        () -> {
          mode = "off";
        });
  }

  public Command reverseHopper() {
    return runOnce(
        () -> {
          mode = "reverse";
        });
  }

  public Command altMode(String shooterMode){
    return runOnce(
      () -> {
        if(shooterMode == "off"){
          mode = "forward";
        }else{
          mode = "off";
        }
      }
    );
  }

  public Command regMode(String shooterMode){
    return runOnce(
      () -> {
        if(shooterMode == "off"){
          mode = "off";
        } else {
          mode = "forward";
        }
      }
    );
  }

  @Override
  public void periodic() {

    if(mode == "forward"){
      setPoint = -2500;
    } else if(mode == "reverse"){
      setPoint = 1800;
    } else {
      setPoint = 0;
    }
    hopperController.setSetpoint(setPoint, ControlType.kVelocity);

    Logger.recordOutput("Hopper/mode", mode);
    Logger.recordOutput("Hopper/Output", hopperMotor.getAppliedOutput());
    Logger.recordOutput("Hopper/speed", hopperMotor.getEncoder().getVelocity());
    Logger.recordOutput("Hopper/setPoint", setPoint);
  }
}
