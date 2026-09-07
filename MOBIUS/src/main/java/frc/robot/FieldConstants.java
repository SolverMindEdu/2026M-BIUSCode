// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

public final class FieldConstants {
  private FieldConstants() {}

  private static final int[] kBlueGoalTagIds = {18, 19, 20, 21, 24, 25, 26, 27};

  private static final int[] kRedGoalTagIds = {2, 3, 4, 5, 8, 9, 10, 11};

  public static final Translation2d kBlueGoal = centroidOf(kBlueGoalTagIds);
  public static final Translation2d kRedGoal = centroidOf(kRedGoalTagIds);

  private static Alliance lastKnownAlliance = Alliance.Blue;

  public static Translation2d ourGoal() {
    DriverStation.getAlliance().ifPresent(alliance -> lastKnownAlliance = alliance);
    return lastKnownAlliance == Alliance.Red ? kRedGoal : kBlueGoal;
  }

  private static Translation2d centroidOf(int[] tagIds) {
    double x = 0.0;
    double y = 0.0;
    for (int id : tagIds) {
      var pose = VisionConstants.aprilTagLayout.getTagPose(id).orElseThrow();
      x += pose.getX();
      y += pose.getY();
    }
    return new Translation2d(x / tagIds.length, y / tagIds.length);
  }
}
