// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  private Constants() {}

  // Publishes every LoggedTunableNumber to /Tuning so gains can be changed live from
  // AdvantageScope. Turn OFF for competition -- the values become fixed and the traffic goes away.
  public static final boolean kTuningMode = true;

  private static final Mode kSimMode = Mode.SIM;

  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final class OperatorConstants {
    private OperatorConstants() {}

    public static final int kDriverControllerPort = 0;
  }
}
