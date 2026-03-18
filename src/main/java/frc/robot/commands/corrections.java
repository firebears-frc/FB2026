package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class corrections {
  // ~CONSTANTS~in meters / radians
  static final double shooterXOffset = Units.inchesToMeters(-5);
  static final double shooterYOffset = Units.inchesToMeters(6);
  static final double shooterAngleOffset = Units.degreesToRadians(90);
  private static boolean doDrawShotLine =
      false; // Do we want to log the line from shooter to target?

  // Returns a boolean for if the shooter is aimed at the hub if on our side, the nearest bumper if
  // in any other zone
  public static boolean aimedAtAutoTarget(Drive drive) {
    boolean aimedAtTarget = false;
    if (currentZone(drive) <= 0) {
      aimedAtTarget =
          Math.abs(angleToHub(drive).getDegrees() - drive.getPose().getRotation().getDegrees()) < 3;
    } else {
      aimedAtTarget =
          Math.abs(
                  angleToNearestBump(drive).getDegrees()
                      - drive.getPose().getRotation().getDegrees())
              < 3;
    }
    Logger.recordOutput("corrections/aimed at target", aimedAtTarget);
    return aimedAtTarget;
  }
  // Returns the angle from the shooter to the hub for autoaim if in alliance zone, returns the
  // angle from the shooter to the nearest bump otherwise
  public static Rotation2d autoAimAngle(Drive drive) {
    if (currentZone(drive) <= 0) {
      return angleToHub(drive);
    } else {
      return angleToNearestBump(drive);
    }
  }
  // Returns the angle from the shooter to the hub
  public static Rotation2d angleToHub(Drive drive) {
    double hubX = correctXValue(LinesVertical.hubCenter);
    double hubY = LinesHorizontal.center;
    Rotation2d angleToHub =
        angleTo(drive, hubX, hubY, shooterXOffset, shooterYOffset, shooterAngleOffset);
    Logger.recordOutput("corrections/angle to hub", angleToHub);
    return angleToHub;
  }

  public static Rotation2d angleTo(
      Drive drive,
      double targetX,
      double targetY,
      double componentX,
      double componentY,
      double componentAngle) {
    return corrections.makeRotation2D(
        corrections.correctAngleForComponent(
            corrections.correctAngleValue(
                Math.atan(
                    Math.abs(
                            (targetY
                                - corrections.yValueOfComponent(componentX, componentY, drive)))
                        / Math.abs(
                            (targetX
                                - corrections.xValueOfComponent(componentX, componentY, drive)))),
                targetX,
                targetY,
                componentX,
                componentY,
                drive),
            componentAngle));
  }

  // Returns the angle to the center of the nearest bump dividing your alliance zone and the center
  public static Rotation2d angleToNearestBump(Drive drive) {
    double nearestBumpY = 0;
    double nearestBumpX = correctXValue(LinesVertical.hubCenter);
    if (drive.getPose().getY() > LinesHorizontal.center) {
      nearestBumpY = (LinesHorizontal.leftBumpStart + LinesHorizontal.leftBumpEnd) / 2;
    } else {
      nearestBumpY = (LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2;
    }
    Rotation2d angleToBump =
        angleTo(
            drive, nearestBumpX, nearestBumpY, shooterXOffset, shooterYOffset, shooterAngleOffset);
    Logger.recordOutput("corrections/angle to bump", angleToBump);
    return angleToBump;
  }

  // Returns the zone the robot is currently in. 0 = alliance side, 1 = center, 2 = opposing side.
  // -1 = failed to find zone.
  public static int currentZone(Drive drive) {
    int currentZone = -1;
    double currentX = drive.getPose().getX();
    if (currentX < LinesVertical.hubCenter) {
      if (onRedAlliance()) {
        currentZone = 2;
      } else {
        currentZone = 0;
      }
    } else if (currentX > LinesVertical.oppHubCenter) {
      if (onRedAlliance()) {
        currentZone = 0;
      } else {
        currentZone = 2;
      }
    } else if (currentX < LinesVertical.oppHubCenter && currentX > LinesVertical.hubCenter) {
      currentZone = 1;
    }
    Logger.recordOutput("corrections/currentZone", currentZone);
    return currentZone;
  }

  // Decide whether or not we want to draw the shotline
  public static void setDrawShotLine(boolean draw) {
    doDrawShotLine = draw;
  }

  // Log a line segment from the shooter, in the shooter direction of length distance_to_hub
  private static void logShotLine(Drive drive, double distanceToHub) {

    // Get current robot pose
    Pose2d robotPose = drive.getPose();

    // Specify the offset from the shooter to the robot (so we can use it to find shooter pose)
    Transform2d shooterOffset =
        new Transform2d(
            new Translation2d(shooterXOffset, shooterYOffset),
            new Rotation2d(-1 * shooterAngleOffset));

    // Find pose of the shooter
    Pose2d shooterPose = robotPose.transformBy(shooterOffset);

    // Using current position shooter, and the distance to the hub, find the end point
    Translation2d lineEndTranslation =
        shooterPose
            .getTranslation()
            .plus(new Translation2d(distanceToHub, 0).rotateBy(shooterPose.getRotation()));

    // Get full pose of our target destination for the shooter
    Pose2d endPose = new Pose2d(lineEndTranslation, shooterPose.getRotation());

    // Log the shotline as well as the shooter location on the bot
    Logger.recordOutput("Shooter/ShotLine", new Pose2d[] {shooterPose, endPose});
    Logger.recordOutput("Shooter/Marker", new Pose2d[] {shooterPose});
  }

  // Gets the distance from the robots current location to the hub
  public static double distanceToHub(Drive drive) {
    double distance =
        distanceTo(drive, correctXValue(LinesVertical.hubCenter), LinesHorizontal.center);

    Logger.recordOutput("Odometry/distance to hub", distance);

    // optionally log the shotline and the shooter position on the bot
    if (doDrawShotLine) {
      logShotLine(drive, distance);
    } else {
      Logger.recordOutput("Shooter/ShotLine", new Pose2d[] {});
      Logger.recordOutput("Shooter/Marker", new Pose2d[] {});
    }
    return distance;
  }

  // Gets the distance from the robot to a specified X and Y
  public static double distanceTo(Drive drive, double X, double Y) {
    double xDifference = drive.getPose().getX() - X;
    double yDifference = drive.getPose().getY() - Y;
    double distance = Math.sqrt((xDifference * xDifference) + (yDifference * yDifference));
    return distance;
  }

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
  public static Rotation2d nearestDiagonalAngle(Drive drive) {
    double newAngle = 0;
    if (Math.abs(drive.getPose().getRotation().getRadians() - (Math.PI / 4)) <= (Math.PI / 4)) {
      newAngle = Math.PI / 4;
    } else if (Math.abs(drive.getPose().getRotation().getRadians() - (-Math.PI / 4))
        <= (Math.PI / 4)) {
      newAngle = -Math.PI / 4;
    } else if (Math.abs(drive.getPose().getRotation().getRadians() - (3 * Math.PI / 4))
        <= (Math.PI / 4)) {
      newAngle = 3 * Math.PI / 4;
    } else if (Math.abs(drive.getPose().getRotation().getRadians() - (-3 * Math.PI / 4))
        <= (Math.PI / 4)) {
      newAngle = -3 * Math.PI / 4;
    }

    return makeRotation2D(newAngle);
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

  // Corrects the angle towards a target based on the bots position around the target, starting
  // angle as if in
  public static double correctAngleValue(
      double angleValue,
      double targetLocationX,
      double targetLocationY,
      double offsetX,
      double offsetY,
      Drive drive) {
    double newAngleValue = angleValue;

    if (xValueOfComponent(offsetX, offsetY, drive) > targetLocationX) {
      newAngleValue = Math.PI - newAngleValue;
    }

    newAngleValue = makeAngleInBounds(newAngleValue);

    if (yValueOfComponent(offsetX, offsetY, drive) > targetLocationY) {
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

  // Makes an angle between -180 and 180
  public static double makeAngleInBoundsDegrees(double Angle) {
    double newAngle = Angle;
    while (newAngle > 180) {
      newAngle -= 360;
    }

    while (newAngle < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}
