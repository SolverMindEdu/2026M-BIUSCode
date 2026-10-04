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

  /** The side-wall tags beside each hub; the alliance-wall tags are left out on purpose. */
  private static final int[] kTrenchTagIds = {1, 6, 7, 12, 17, 22, 23, 28};

  public static final Translation2d kBlueGoal = centroidOf(kBlueGoalTagIds);
  public static final Translation2d kRedGoal = centroidOf(kRedGoalTagIds);

  private static Alliance lastKnownAlliance = Alliance.Blue;

  /**
   * Whether that default has ever been replaced by a real one. Everything that mirrors with the
   * alliance -- which goal is ours, which half is our alliance zone -- is a guess until the driver
   * station says, and a guess on the wrong alliance points the turret at the other end of the
   * field. Anything that acts on the alliance rather than merely reporting it checks this first.
   */
  private static boolean allianceKnown = false;

  // Pass placements are given in blue-alliance coordinates; red's pair is these rotated. Both sit
  // about a metre off their side wall and a metre off the alliance wall, well inside our own
  // ALLIANCE ZONE, whose boundary is the hub line about 4 m out.

  /** Where a pass lands when it goes to the driver's left, in blue-alliance coordinates. */
  private static final Translation2d kBluePassLeft = new Translation2d(1.0, 7.0);

  /** Where a pass lands when it goes to the driver's right, in blue-alliance coordinates. */
  private static final Translation2d kBluePassRight = new Translation2d(1.0, 1.0);

  /** Hub and trench tags only; the alliance-wall tags do not move the pose the turret aims from. */
  public static boolean isAimTag(int id) {
    return contains(kBlueGoalTagIds, id) || contains(kRedGoalTagIds, id) || contains(kTrenchTagIds, id);
  }

  private static boolean contains(int[] ids, int id) {
    for (int candidate : ids) {
      if (candidate == id) {
        return true;
      }
    }
    return false;
  }

  public static Alliance alliance() {
    DriverStation.getAlliance()
        .ifPresent(
            alliance -> {
              lastKnownAlliance = alliance;
              allianceKnown = true;
            });
    return lastKnownAlliance;
  }

  /**
   * True once the driver station has actually named an alliance. It latches, so a dropped station
   * mid-match keeps the alliance we were given rather than falling back to the default.
   */
  public static boolean allianceKnown() {
    alliance();
    return allianceKnown;
  }

  public static Translation2d ourGoal() {
    return alliance() == Alliance.Red ? kRedGoal : kBlueGoal;
  }

  /** How far past the hub line, toward our own wall, the robot must be to count as back over the bump. */
  private static final double kAllianceZoneMarginMeters = 0.5;

  /** True once the robot is back over the bump in our own alliance zone. */
  public static boolean inOurAllianceZone(Translation2d position) {
    double hubX = ourGoal().getX();
    return alliance() == Alliance.Red
        ? position.getX() > hubX + kAllianceZoneMarginMeters
        : position.getX() < hubX - kAllianceZoneMarginMeters;
  }

  /** BOTH lets the robot's own position choose the side; LEFT and RIGHT pin it. */
  public enum PassSide {
    BOTH("Both"),
    LEFT("Left"),
    RIGHT("Right");

    /** The dashboard button name, tied to the constant so the two cannot drift apart. */
    public final String label;

    PassSide(String label) {
      this.label = label;
    }
  }

  /**
   * Red's copy of a point given in blue-alliance coordinates.
   *
   * <p>The 2026 field is rotationally symmetric, not mirrored across the centre line, so the map is
   * a 180-degree turn about the field centre and Y flips along with X. Checked against the tag
   * layout: every off-centre tag has a partner at (length - x, width - y) and none has one at
   * (length - x, y). The outposts show it plainly -- blue's tags 29/30 sit near y = 0.67 and 1.10
   * while red's 13/14 sit near y = 7.40 and 6.97, and each pair sums to the field width.
   */
  public static Translation2d mirrorForAlliance(Translation2d bluePoint) {
    if (alliance() != Alliance.Red) {
      return bluePoint;
    }
    return new Translation2d(
        VisionConstants.aprilTagLayout.getFieldLength() - bluePoint.getX(),
        VisionConstants.aprilTagLayout.getFieldWidth() - bluePoint.getY());
  }

  /** Lands the pass deep in our own zone, on the chosen side of the field. */
  public static Translation2d passTarget(Translation2d from, PassSide side) {
    // Left and right are the driver's, looking downfield. The pair is written for blue and turned
    // for red, so the side the driver names is the side the pass goes to on either alliance.
    boolean driverLeft =
        switch (side) {
          case BOTH -> isDriverLeft(from);
          case LEFT -> true;
          case RIGHT -> false;
        };
    return mirrorForAlliance(driverLeft ? kBluePassLeft : kBluePassRight);
  }

  /** Which side of the field a point is on, from the driver's view rather than in field Y. */
  public static boolean isDriverLeft(Translation2d position) {
    boolean highY = position.getY() > VisionConstants.aprilTagLayout.getFieldWidth() / 2.0;
    return alliance() == Alliance.Red ? !highY : highY;
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
