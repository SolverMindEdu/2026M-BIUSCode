// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import java.util.OptionalDouble;

public final class TurretMath {
  private TurretMath() {}

  public static OptionalDouble resolveSetpoint(
      double desiredDegrees, double currentDegrees, double minDegrees, double maxDegrees) {
    return resolveSetpoint(desiredDegrees, currentDegrees, minDegrees, maxDegrees, currentDegrees);
  }

  /** Picks the legal wrap nearest referenceDegrees, so a committed move is not re-decided mid-slew. */
  public static OptionalDouble resolveSetpoint(
      double desiredDegrees,
      double currentDegrees,
      double minDegrees,
      double maxDegrees,
      double referenceDegrees) {
    double nearest =
        currentDegrees + MathUtil.inputModulus(desiredDegrees - currentDegrees, -180.0, 180.0);

    double best = Double.NaN;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (int wrap = -2; wrap <= 2; wrap++) {
      double candidate = nearest + wrap * 360.0;
      if (candidate < minDegrees || candidate > maxDegrees) {
        continue;
      }
      double distance = Math.abs(candidate - referenceDegrees);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = candidate;
      }
    }

    return Double.isNaN(best) ? OptionalDouble.empty() : OptionalDouble.of(best);
  }

  public static double referenceError(double measuredDegrees, double referenceDegrees) {
    return MathUtil.inputModulus(measuredDegrees - referenceDegrees, -180.0, 180.0);
  }

  public static double clampToNearestLimit(
      double desiredDegrees, double minDegrees, double maxDegrees) {
    double toMax = Math.abs(MathUtil.inputModulus(desiredDegrees - maxDegrees, -180.0, 180.0));
    double toMin = Math.abs(MathUtil.inputModulus(desiredDegrees - minDegrees, -180.0, 180.0));
    return toMax <= toMin ? maxDegrees : minDegrees;
  }
}
