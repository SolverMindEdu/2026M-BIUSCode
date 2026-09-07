// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableNumber implements DoubleSupplier {
  private static final String kTableKey = "/Tuning";

  private final String key;
  private final double defaultValue;
  private final LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastValues = new HashMap<>();

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.key = kTableKey + "/" + dashboardKey;
    this.defaultValue = defaultValue;
    this.dashboardNumber = Constants.kTuningMode ? new LoggedNetworkNumber(key, defaultValue) : null;
  }

  public double get() {
    return dashboardNumber == null ? defaultValue : dashboardNumber.get();
  }

  public boolean hasChanged(int id) {
    double current = get();
    Double last = lastValues.get(id);
    if (last == null || current != last) {
      lastValues.put(id, current);
      return true;
    }
    return false;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... numbers) {
    if (Arrays.stream(numbers).anyMatch(n -> n.hasChanged(id))) {
      action.accept(Arrays.stream(numbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
