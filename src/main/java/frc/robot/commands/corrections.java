package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.subsystems.drive.Drive;

public class corrections {

  // Corrects a x value by flipping it over the center line if and only if current alliance is red.
  public static double correctXValue(double xValue) {
    if (onRedAlliance()) {
      return flipX(xValue);
    } else {
      return xValue;
    }
  }

  // Corrects a y value by flipping it over the center line if and only if current alliance is red.
  public static double correctYValue(double yValue) {
    if (onRedAlliance()) {
      return flipY(yValue);
    } else {
      return yValue;
    }
  }

  // Corrects an angle value by changing it PI radians if and only if current alliance is red.
  public static double correctAngleValue(double angleValue) {
    double newAngle = angleValue;
    if (onRedAlliance()) {
      newAngle += Math.PI;
    }
    newAngle = makeAngleInBounds(newAngle);
    return newAngle;
  }

  // Flips a x value around the center line.
  private static double flipX(double xValue) {
    return flipValueAround(xValue, LinesVertical.center);
  }

  // Flips a y value around the center line.
  private static double flipY(double yValue) {
    return flipValueAround(yValue, LinesHorizontal.center);
  }

  // Flips a value around another value
  private static double flipValueAround(double value, double flipAroundValue) {
    double newValue = value;
    newValue -= flipAroundValue;
    newValue *= -1;
    newValue += flipAroundValue;
    return newValue;
  }

  // returns the nearest PI / 2 radian angle to the bots current position
  public static double nearestDiagonalAngle(Drive drive) {
    double newAngle = 0;
    if (Math.abs(drive.getPose().getRotation().getRadians() - (Math.PI / 2)) <= (Math.PI / 2)) {
      newAngle = Math.PI / 2;
    } else if (Math.abs(drive.getPose().getRotation().getRadians() - (-Math.PI / 2))
        <= (Math.PI / 2)) {
      newAngle = -Math.PI / 2;
    } else if (Math.abs(drive.getPose().getRotation().getRadians() - (3 * Math.PI / 2))
        <= (Math.PI / 2)) {
      newAngle = 3 * Math.PI / 2;
    } else if (Math.abs(drive.getPose().getRotation().getRadians() - (-3 * Math.PI / 2))
        <= (Math.PI / 2)) {
      newAngle = -3 * Math.PI / 2;
    }

    return newAngle;
  }

  // Corrects the X for the location of a part of the bot given an offset between it and the center
  // of the bot
  // Offsets are based on the offsets when the bot is at angle 0, following field rules for positive
  // and negative.
  public static double xValueOfComponent(double offsetX, double offsetY, Drive drive) {
    double newLocation = drive.getPose().getX();
    newLocation += Math.cos(drive.getPose().getRotation().getRadians()) * offsetX;
    newLocation -= Math.sin(drive.getPose().getRotation().getRadians()) * offsetY;
    return newLocation;
  }

  // Corrects the Y for the location of a part of the bot given an offset between it and the center
  // of the bot
  // Offsets are based on the offsets when the bot is at angle 0, following field rules for positive
  // and negative.
  public static double yValueOfComponent(double offsetX, double offsetY, Drive drive) {
    double newLocation = drive.getPose().getY();
    newLocation += Math.cos(drive.getPose().getRotation().getRadians()) * offsetY;
    newLocation += Math.sin(drive.getPose().getRotation().getRadians()) * offsetX;
    return newLocation;
  }

  // Corrects the angle the bot needs to face to make a component face the original angle
  public static double correctAngleForComponent(double oldAngle, double offset) {
    double newAngle = oldAngle;

    newAngle += offset;

    newAngle = makeAngleInBounds(newAngle);

    return newAngle;
  }

  // Corrects the angle towards a target based on the bots position around the target
  public static double correctAngleValue(
      double angleValue, double targetLocationX, double targetLocationY, Drive drive) {
    double newAngleValue = angleValue;

    if (drive.getPose().getX() > targetLocationX) {
      newAngleValue = Math.PI - newAngleValue;
    }

    newAngleValue = makeAngleInBounds(newAngleValue);

    if (drive.getPose().getY() > targetLocationY) {
      newAngleValue *= -1;
    }

    return newAngleValue;
  }

  // Creates a new rotation2d from an angle
  public static Rotation2d makeRotation2D(double rotation) {
    Rotation2d newRotation2d = new Rotation2d(rotation);
    return newRotation2d;
  }

  // Returns true if on red alliance
  public static boolean onRedAlliance() {
    boolean redAlliance = false;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        redAlliance = true;
      }
    }
    return redAlliance;
  }

  // Makes an angle between PI and -PI
  private static double makeAngleInBounds(double Angle) {
    double newAngle = Angle;
    while (newAngle > Math.PI) {
      newAngle -= 2 * Math.PI;
    }

    while (newAngle < -Math.PI) {
      newAngle += 2 * Math.PI;
    }
    return newAngle;
  }
}
