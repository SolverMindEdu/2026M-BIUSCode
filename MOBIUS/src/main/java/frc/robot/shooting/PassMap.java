// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import frc.robot.shooting.ShotTable.ShotParameters;
/** Lob parameters for passing out of the opponent half, keyed the same way as ShotMap. */
public final class PassMap {
  private PassMap() {}

  private static final ShotTable kTable = new ShotTable();

  static {
    // Distance, hood degrees, RPM, flight time in seconds.
    // TODO(10015): still UNMEASURED, but no longer physically silly. The hood is pinned at 237,
    // the furthest angle the shot map has ever actually used, because asking for 250-290 meant
    // hood.atGoal() could never be satisfied and the feed simply never opened. Range past the
    // table therefore comes from RPM alone, scaled as the square root of distance, and the flight
    put(4.0, 237.0, 1840.0, 3.0);
    put(5.0, 237.0, 2060.0, 3.0);
    put(6.0, 237.0, 2260.0, 3.0);
    put(7.0, 237.0, 2440.0, 3.0);
    put(8.5, 237.0, 2690.0, 3.0);
  }

  private static void put(
      double distanceMeters, double hoodDegrees, double shooterRpm, double flightSecs) {
    kTable.put(distanceMeters, new ShotParameters(shooterRpm, hoodDegrees, flightSecs));
  }

  public static ShotTable table() {
    return kTable;
  }

  public static ShotMap.Shot lookup(double distanceMeters) {
    var parameters = kTable.get(distanceMeters);
    return new ShotMap.Shot(parameters.hoodDegrees(), parameters.rpm());
  }
}
