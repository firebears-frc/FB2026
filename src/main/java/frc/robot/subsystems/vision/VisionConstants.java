// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera1Name = "VisionCam1";
  public static String camera2Name = "VisionCam2";
  public static String camera4Name = "VisionCam4"; // Back Right
  public static String camera6Name = "VisionCam6"; // Front Right
  public static String camera7Name = "VisionCam7"; // Front Left
  public static String camera8Name = "VisionCam8"; // Back Left
  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera1 =
      new Transform3d(0, 0.3, 0.2, new Rotation3d(0.0, 0.4, Math.PI / 2));
  public static Transform3d robotToCamera2 =
      new Transform3d(0, 0.13, 0.2, new Rotation3d(0.0, 0.4, Math.PI / 2));
  public static Transform3d robotToCamera4 =
      new Transform3d(
          Units.inchesToMeters(10.75),
          Units.inchesToMeters(-12.625),
          Units.inchesToMeters(14.25),
          new Rotation3d(
              Units.degreesToRadians(12), Units.degreesToRadians(-4), Units.degreesToRadians(296)));
  public static Transform3d robotToCamera6 =
      new Transform3d(
          Units.inchesToMeters(-11.5),
          Units.inchesToMeters(-12.5625),
          Units.inchesToMeters(14),
          new Rotation3d(0, Units.degreesToRadians(-18), Units.degreesToRadians(225)));
  public static Transform3d robotToCamera7 =
      new Transform3d(
          Units.inchesToMeters(8.625),
          Units.inchesToMeters(13.875),
          Units.inchesToMeters(14.625),
          new Rotation3d(0, 0, Units.degreesToRadians(90)));
  public static Transform3d robotToCamera8 =
      new Transform3d(
          Units.inchesToMeters(-11.75),
          Units.inchesToMeters(12.875),
          Units.inchesToMeters(14.3125),
          new Rotation3d(0, Units.degreesToRadians(-21), Units.degreesToRadians(135)));
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, 1.0, 1.0, 1.0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
