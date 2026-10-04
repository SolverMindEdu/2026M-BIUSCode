// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class PhoenixUtil {
  private PhoenixUtil() {}

  public static InvertedValue direction(boolean inverted) {
    return inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  }

  /** Configures a coasting, current-limited roller and returns it ready to use. */
  public static TalonFX configureRoller(
      int id, CANBus bus, boolean inverted, double statorAmps, double supplyAmps) {
    return configureRoller(id, bus, inverted, statorAmps, supplyAmps, supplyAmps, 0.0);
  }

  /**
   * The same, with a supply limit that backs off once it has been held for a while.
   *
   * <p>Phoenix allows the supply up to supplyAmps, and if the limit has been active continuously
   * for supplyLowerTimeSecs it drops the ceiling to supplyLowerAmps. That separates the two things
   * a roller needs: enough current to break a stalled column loose, and a sustained draw the
   * battery can carry once it is clear that the motor is not going to win. A roller that is simply
   * working never reaches the limit and never sees the reduction.
   *
   * <p>Pass supplyLowerAmps equal to supplyAmps with a zero time for a flat limit.
   */
  public static TalonFX configureRoller(
      int id,
      CANBus bus,
      boolean inverted,
      double statorAmps,
      double supplyAmps,
      double supplyLowerAmps,
      double supplyLowerTimeSecs) {
    TalonFX motor = new TalonFX(id, bus);
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = statorAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = supplyAmps;
    config.CurrentLimits.SupplyCurrentLowerLimit = supplyLowerAmps;
    config.CurrentLimits.SupplyCurrentLowerTime = supplyLowerTimeSecs;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = direction(inverted);
    motor.getConfigurator().apply(config);
    return motor;
  }

  /** Subscribes a roller's telemetry at 50 Hz and drops everything else off the bus. */
  public static void publishRollerSignals(TalonFX motor, StatusSignal<?>... signals) {
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals);
    motor.optimizeBusUtilization();
  }

  /**
   * Rate for signals nothing controls off -- power accounting and the like. Deliberately slow:
   * ranking what a mechanism costs needs nothing like loop rate, and the RIO is already overrunning
   * without spending bus and CPU on frames only the log reads. Call before optimizeBusUtilization,
   * which drops any signal that has not been asked for.
   */
  public static final double kMonitorHz = 20.0;

  /** Sets every given signal to the monitoring rate. */
  public static void publishMonitorSignals(StatusSignal<?>... signals) {
    BaseStatusSignal.setUpdateFrequencyForAll(kMonitorHz, signals);
  }
}
