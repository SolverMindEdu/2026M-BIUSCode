// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static boolean kVisionEnabled = true;

  public static boolean kLogPerCameraPoses = false;

  public static String camera0Name = "limelight-side";
  public static String camera1Name = "limelight-front";
  public static String camera2Name = "limelight-back";

  public static double poseVisionTimeoutSecs = 0.25;

  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  public static double linearStdDevBaseline = 0.06;
  public static double angularStdDevBaseline = Double.POSITIVE_INFINITY;

  public static double[] cameraStdDevFactors =
      new double[] {
        1.0,
        1.0,
        1.0
      };

  public static double linearStdDevMegatag2Factor = 0.5;
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY;
}
