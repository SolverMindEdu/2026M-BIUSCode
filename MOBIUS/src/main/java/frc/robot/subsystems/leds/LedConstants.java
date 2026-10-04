// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public final class LedConstants {
  private LedConstants() {}

  /** 40 is clear of every motor on the roboRIO bus, which run 0 to 30. */
  public static final int kCandleId = 40;

  public static final CANBus kCanBus = CANBus.roboRIO();

  /** 8 onboard CANdle LEDs plus a 24-LED strip. */
  public static final int kLedCount = 32;

  public static final double kBrightness = 0.5;

  /** Byte order the strip expects; the onboard LEDs ignore this, so only the strip looks wrong. */
  public static final StripTypeValue kStripType = StripTypeValue.RGB;

  /** Shown from power-on until the robot is enabled. */
  public static final RGBWColor kStartupColor = new RGBWColor(55, 145, 225);

  /** Fuel is actually going through. */
  public static final RGBWColor kFeedingColor = new RGBWColor(0, 255, 0);

  /** Turret is on target, so a shot would go now. */
  public static final RGBWColor kReadyColor = new RGBWColor(255, 255, 0);

  public static final RGBWColor kNotAlignedColor = new RGBWColor(255, 0, 0);

  /** Strobes per second while ready; high enough to read as a rapid blink. */
  public static final double kReadyStrobeHz = 8.0;
}
