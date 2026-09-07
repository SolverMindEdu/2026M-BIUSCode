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
    TalonFX motor = new TalonFX(id, bus);
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = statorAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = supplyAmps;
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
}
