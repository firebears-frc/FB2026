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
  public static String Camera0 = "Camera0"; // Front Left
  public static String Camera1 = "Camera1"; // Back Left
  public static String Camera2 = "Camera2"; // Front Right
  public static String Camera3 = "Camera3"; // Back Right
  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(8.5),
          Units.inchesToMeters(13.6275),
          Units.inchesToMeters(13.5625),
          new Rotation3d(
              Units.degreesToRadians(12), Units.degreesToRadians(-4), Units.degreesToRadians(296)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-11.25),
          Units.inchesToMeters(12.9375),
          Units.inchesToMeters(16.625),
          new Rotation3d(0, Units.degreesToRadians(-18), Units.degreesToRadians(225)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(11),
          Units.inchesToMeters(-12.375),
          Units.inchesToMeters(14.375),
          new Rotation3d(0, 0, Units.degreesToRadians(90)));
  public static Transform3d robotToCamera3 =
      new Transform3d(
          Units.inchesToMeters(-11.5),
          Units.inchesToMeters(-12.75),
          Units.inchesToMeters(16.625),
          new Rotation3d(0, Units.degreesToRadians(-26), Units.degreesToRadians(135)));
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
