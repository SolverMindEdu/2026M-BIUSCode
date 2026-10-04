// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LedConstants.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {
  private final CANdle candle = new CANdle(kCandleId, kCanBus);
  private final SolidColor solidRequest = new SolidColor(0, kLedCount - 1);
  private final StrobeAnimation strobeRequest = new StrobeAnimation(0, kLedCount - 1).withSlot(0);

  /** An animation keeps running in its slot until cleared; a solid colour alone will not stop it. */
  private final EmptyAnimation clearAnimation = new EmptyAnimation(0);

  private String state = "";

  public Leds() {
    var config = new CANdleConfiguration();
    config.LED.BrightnessScalar = kBrightness;
    config.LED.StripType = kStripType;
    candle.getConfigurator().apply(config);

    // Set in the constructor, not a command: the lights come up with the robot, while disabled.
    solid(kStartupColor, "Startup");
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Leds/Connected", candle.isConnected());
    Logger.recordOutput("Leds/State", state);
  }

  public boolean isConnected() {
    return candle.isConnected();
  }

  /** Requests are sent only when the state changes, so a held colour costs no CAN traffic. */
  public void solid(RGBWColor color, String name) {
    if (state.equals(name)) {
      return;
    }
    state = name;
    candle.setControl(clearAnimation);
    candle.setControl(solidRequest.withColor(color));
  }

  public void strobe(RGBWColor color, double frameRateHz, String name) {
    if (state.equals(name)) {
      return;
    }
    state = name;
    candle.setControl(strobeRequest.withColor(color).withFrameRate(frameRateHz));
  }
}
