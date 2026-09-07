// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShotMap {
  private ShotMap() {}

  public record Shot(double hoodDegrees, double shooterRpm) {}

  private static final InterpolatingDoubleTreeMap kHoodDegrees = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap kShooterRpm = new InterpolatingDoubleTreeMap();

  static {
    // TODO(10015): one row per measured distance. Park the robot, read

    put(2.0, 200.0, 1000.0);
  }

  private static void put(double distanceMeters, double hoodDegrees, double shooterRpm) {
    kHoodDegrees.put(distanceMeters, hoodDegrees);
    kShooterRpm.put(distanceMeters, shooterRpm);
  }

  public static Shot lookup(double distanceMeters) {
    return new Shot(kHoodDegrees.get(distanceMeters), kShooterRpm.get(distanceMeters));
  }
}
