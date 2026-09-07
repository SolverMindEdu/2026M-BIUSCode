// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTable {
  public record ShotParameters(double rpm, double hoodDegrees, double timeOfFlightSecs)
      implements Interpolatable<ShotParameters> {
    @Override
    public ShotParameters interpolate(ShotParameters endValue, double t) {
      return new ShotParameters(
          MathUtil.interpolate(rpm, endValue.rpm, t),
          MathUtil.interpolate(hoodDegrees, endValue.hoodDegrees, t),
          MathUtil.interpolate(timeOfFlightSecs, endValue.timeOfFlightSecs, t));
    }
  }

  private final InterpolatingTreeMap<Double, ShotParameters> map =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters::interpolate);

  private double minDistanceMeters = Double.POSITIVE_INFINITY;
  private double maxDistanceMeters = Double.NEGATIVE_INFINITY;

  public void put(double distanceMeters, ShotParameters parameters) {
    map.put(distanceMeters, parameters);
    minDistanceMeters = Math.min(minDistanceMeters, distanceMeters);
    maxDistanceMeters = Math.max(maxDistanceMeters, distanceMeters);
  }

  public ShotParameters get(double distanceMeters) {
    ShotParameters parameters = map.get(distanceMeters);
    if (parameters == null) {
      throw new IllegalStateException("ShotTable has no entries; cannot solve a shot.");
    }
    return parameters;
  }

  public boolean isWithinRange(double distanceMeters) {
    return distanceMeters >= minDistanceMeters && distanceMeters <= maxDistanceMeters;
  }}
