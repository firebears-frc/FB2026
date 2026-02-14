package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.subsystems.drive.Drive;

public class corrections {

  // Corrects an x value by flipping it over the center line if and only if current alliance is red.
  public static double correctXValue(double xValue) {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return flipAroundCenter(xValue);
    } else {
      return xValue;
    }
  }

  // Flips a x value around the center line.
  private static double flipAroundCenter(double xValue) {
    double newValue = xValue;
    newValue -= LinesVertical.center;
    newValue *= -1;
    newValue += LinesVertical.center;
    return newValue;
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
