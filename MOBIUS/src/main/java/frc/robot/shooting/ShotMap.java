// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import frc.robot.shooting.ShotTable.ShotParameters;

public final class ShotMap {
  private ShotMap() {}

  public record Shot(double hoodDegrees, double shooterRpm) {}

  private static final ShotTable kTable = new ShotTable();

  static {
    // Distance, hood degrees, RPM, flight time in seconds.
    // TODO(10015): 1.0-second flight times are UNMEASURED estimates. Measure exit-to-hub
    // time at each distance before relying on moving shots, including low-battery shots.
    put(1.0, 52.0, 1500.0, 1.14);
    put(1.2, 57.0, 1500.0, 1.10);
    put(1.4, 66.0, 1500.0, 1.055);
    put(1.6, 115.0, 1600.0, 1.05);
    put(1.8, 119.0, 1600.0, 1.17); 
    put(2.0, 128.0, 1600.0, 1.22); 
    put(2.2, 142.0, 1600.0, 1.24);
    put(2.4, 157.0, 1600.0, 1.23);
    put(2.6, 168.0, 1600.0, 1.17);
    put(2.8, 180.0, 1600.0, 1.25);
    put(3.0, 210.0, 1700.0, 1.21);
    put(3.2, 226.0, 1700.0, 1.20);
    put(3.4, 239.0, 1700.0, 1.21);
    put(3.6, 270.0, 1750.0, 1.30);
    put(3.8, 290.0, 1780.0, 1.22);
    put(4.0, 290.0, 2000.0, 1.24);

  }

  private static void put(
      double distanceMeters, double hoodDegrees, double shooterRpm, double flightSecs) {
    kTable.put(distanceMeters, new ShotParameters(shooterRpm, hoodDegrees, flightSecs));
  }

  public static ShotTable table() {
    return kTable;
  }

  public static Shot lookup(double distanceMeters) {
    var parameters = kTable.get(distanceMeters);
    return new Shot(parameters.hoodDegrees(), parameters.rpm());
  }
}
