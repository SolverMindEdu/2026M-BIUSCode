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

  /** How long a two-tag sighting keeps the turret on full gains after the tags drop out of view. */
  public static double multiTagTimeoutSecs = 0.25;

  /** Only hub and trench tags may move the pose; alliance-wall tags are ignored. */
  public static boolean kRequireAimTags = true;

  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;


  /** MegaTag2 translation trust, metres; MegaTag1 translation is never used. */
  public static double linearStdDevBaseline = 0.06;

  /** MegaTag1 heading trust, radians; MegaTag2 heading is never used, since it is the gyro's own yaw. */
  public static double angularStdDevBaseline = 0.1;

  /** While enabled, MegaTag1 heading is trusted this many times less, since a flipped heading swings the turret. */
  public static double megatag1EnabledAngularFactor = 10.0;

  /** While enabled, MegaTag1 heading is only used below this ambiguity. */
  public static double megatag1MaxAmbiguity = 0.2;

  /** While enabled, MegaTag1 heading is only used within this range of the tags. */
  public static double megatag1MaxDistanceMeters = 3.0;

  public static double[] cameraStdDevFactors =
      new double[] {
        1.0,
        1.0,
        1.0
      };

  public static double linearStdDevMegatag2Factor = 0.5;

  /** In defense mode MegaTag2 translation is trusted this much more, since a shove moves the robot without turning the wheels. */
  public static double defenseLinearStdDevFactor = 0.5;
}
