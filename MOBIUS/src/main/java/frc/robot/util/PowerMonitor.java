// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Where the battery's current actually goes, broken down by mechanism.
 *
 * <p>Supply current, not stator. Stator is what the motor windings see and is the number the motor
 * controller limits; supply is what the battery delivers, and it is the only one that adds up
 * across mechanisms. A geared mechanism holding position can pull 40 A of stator off 4 A of supply,
 * so ranking by stator would send you optimising the wrong thing.
 *
 * <p>Ranked by charge, not by peak. A flywheel holding 25 A for a whole match costs far more than
 * an intake spiking 90 A for a fifth of a second, but the spike is what stands out on a plot.
 * Peaks are still published because they are what trips a brownout; the two questions have
 * different answers and both are logged.
 *
 * <p>These are motor supply currents only. The RIO, radio, cameras and anything else on the PDH are
 * not included, so the total is what the mechanisms cost, not what the robot draws.
 */
public class PowerMonitor {
  /** A stall this long is a loop that stopped, not current that flowed; do not integrate it. */
  private static final double kMaxStepSecs = 0.1;

  private static final class Entry {
    private double amps;
    private double peakAmps;
    private double ampSeconds;
  }

  private final Map<String, Entry> entries = new LinkedHashMap<>();
  private double lastSecs = Double.NaN;
  private double peakTotalAmps = 0.0;

  /** Call once per mechanism per loop, before {@link #publish()}. */
  public void record(String name, double supplyAmps) {
    entries.computeIfAbsent(name, key -> new Entry()).amps = supplyAmps;
  }

  /** Integrates the loop just recorded and writes the whole breakdown out. */
  public void publish() {
    double now = Timer.getFPGATimestamp();
    double dt = Double.isNaN(lastSecs) ? 0.0 : Math.min(now - lastSecs, kMaxStepSecs);
    lastSecs = now;

    double totalAmps = 0.0;
    double totalCharge = 0.0;
    for (Entry entry : entries.values()) {
      entry.ampSeconds += entry.amps * dt;
      entry.peakAmps = Math.max(entry.peakAmps, entry.amps);
      totalAmps += entry.amps;
      totalCharge += entry.ampSeconds;
    }
    peakTotalAmps = Math.max(peakTotalAmps, totalAmps);

    String largest = "";
    double largestCharge = -1.0;
    for (Map.Entry<String, Entry> keyed : entries.entrySet()) {
      Entry entry = keyed.getValue();
      String prefix = "Power/" + keyed.getKey() + "/";
      Logger.recordOutput(prefix + "Amps", entry.amps);
      Logger.recordOutput(prefix + "PeakAmps", entry.peakAmps);
      Logger.recordOutput(prefix + "AmpSeconds", entry.ampSeconds);
      Logger.recordOutput(
          prefix + "ShareOfCharge", totalCharge > 0.0 ? entry.ampSeconds / totalCharge : 0.0);
      if (entry.ampSeconds > largestCharge) {
        largestCharge = entry.ampSeconds;
        largest = keyed.getKey();
      }
    }

    Logger.recordOutput("Power/TotalAmps", totalAmps);
    Logger.recordOutput("Power/PeakTotalAmps", peakTotalAmps);
    Logger.recordOutput("Power/TotalAmpSeconds", totalCharge);
    Logger.recordOutput("Power/LargestConsumer", largest);
    Logger.recordOutput("Power/BatteryVolts", RobotController.getBatteryVoltage());
    Logger.recordOutput("Power/Brownout", RobotController.isBrownedOut());
  }
}
