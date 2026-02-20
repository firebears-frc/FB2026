package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Arm extends SubsystemBase {
  private static int STALL_CURRENT_LIMIT_SHOULDER = 20;
  private static int FREE_CURRENT_LIMIT_SHOULDER = 20;
  private static double shoulderP = 0.02;
  private static double shoulderI = 0.0;
  private static double shoulderG = 0.35;
  private static double shoulderD = 0.0;
  private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 30;
  private final SparkMax shoulderMotorRight;
  private final SparkAbsoluteEncoder shoulderEncoder;
  private final SparkClosedLoopController shoulderPID;

  @AutoLogOutput(key = "arm/setPoint")
  private Rotation2d shoulderSetpoint = new Rotation2d();

  private final LoggedNetworkNumber shootAngle = new LoggedNetworkNumber("arm/shootAngle", 13.5);

  private Debouncer debounce = new Debouncer(0.2);

  public Arm() {
    shoulderMotorRight = new SparkMax(10, MotorType.kBrushless);
    shoulderEncoder = shoulderMotorRight.getAbsoluteEncoder();
    shoulderPID = shoulderMotorRight.getClosedLoopController();

    var shoulderMotorRightConfig = new SparkMaxConfig();
    shoulderMotorRightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER)
        .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
    shoulderMotorRightConfig.absoluteEncoder.inverted(true).positionConversionFactor(360);
    shoulderMotorRightConfig
        .closedLoop
        .pid(shoulderP, shoulderI, shoulderD)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 360)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    SparkUtil.tryUntilOk(
        shoulderMotorRight,
        5,
        () ->
            shoulderMotorRight.configure(
                shoulderMotorRightConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    

    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    setShoulderSetpoint(getShoulderAngle());
  }

  private static final class Constants {
    private static final Rotation2d armDown = Rotation2d.fromDegrees(0);
    private static final Rotation2d armUp= Rotation2d.fromDegrees(90);
  }

  @AutoLogOutput(key = "arm/Angle")
  public Rotation2d getShoulderAngle() {
    return Rotation2d.fromDegrees(shoulderEncoder.getPosition());
  }

  public void setShoulderSetpoint(Rotation2d setpoint) {
    if (setpoint.getDegrees() < 0) {
      setpoint = Rotation2d.fromDegrees(0);
    } else if (setpoint.getDegrees() > 100) {
      setpoint = Rotation2d.fromDegrees(100);//test robot and then implement correct value
    }
    shoulderSetpoint = setpoint;
  }

  @AutoLogOutput(key = "arm/error")
  private Rotation2d getError() {
    return getShoulderAngle().minus(shoulderSetpoint);
  }

  public Command defaultCommand(Supplier<Double> shoulderChange) {
    return run(
        () -> {
          setShoulderSetpoint(shoulderSetpoint.minus(Rotation2d.fromDegrees(shoulderChange.get())));
        });
  }

  public Command armDown() {
    return positionCommand(() -> Constants.armDown, () -> 1.0);
  }

  public Command armUp() {
    return positionCommand(() -> Constants.armUp, () -> 1.0);
  }
  

  private boolean onTarget(double tolerance) {
    boolean onTarget = Math.abs(getError().getDegrees()) < tolerance;
    Logger.recordOutput("arm/onTargt", onTarget);
    boolean debounced = debounce.calculate(onTarget);
    Logger.recordOutput("arm/at debouncespeed", debounced);
    return debounced;
  }

  private Command positionCommand(Supplier<Rotation2d> position, Supplier<Double> tolerance) {
    return Commands.sequence(
        runOnce(() -> setShoulderSetpoint(position.get())),
        Commands.waitSeconds(0.1),
        run(() -> {}).until(() -> onTarget(tolerance.get())));
  }



  @Override
  public void periodic() {
    double feedForward = Math.cos(getShoulderAngle().getRadians()) * shoulderG;
    shoulderPID.setReference(shoulderSetpoint.getDegrees(), ControlType.kPosition);
    Logger.recordOutput("arm/MotorRight", shoulderMotorRight.getAppliedOutput());
    Logger.recordOutput("arm/MotorRightCurrent", shoulderMotorRight.getOutputCurrent());
    Logger.recordOutput("arm/setPointDegrees", shoulderSetpoint.getDegrees());
    Logger.recordOutput("arm/angleDegrees", getShoulderAngle().getDegrees());
    Logger.recordOutput("arm/FeedForward", feedForward);
  }
}
