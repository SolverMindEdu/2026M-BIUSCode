// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double commandedRpm = 0.0;

  private double offTargetSinceSecs = Double.NaN;

  private double atSpeedSinceSecs = Double.NaN;

  public Shooter(ShooterIO io) {
    this.io = io;
    setDefaultCommand(
        kIdleRpm > 0.0
            ? setRpm(kIdleRpm).withName("ShooterIdle")
            : run(io::stop).withName("ShooterCoast"));
  }

  private static final LoggedTunableNumber tunableP = new LoggedTunableNumber("Shooter/kP", kP);
  private static final LoggedTunableNumber tunableTorque =
      new LoggedTunableNumber("Shooter/PeakTorqueAmps", kPeakTorqueAmps);

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(), values -> io.setGains(values[0], values[1]), tunableP, tunableTorque);
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/AtSpeed", commandedRpm > 0.0 && atSpeed(commandedRpm));
    Logger.recordOutput("Shooter/CommandedRpm", commandedRpm);

    double atSpeedNow = Logger.getTimestamp() / 1.0e6;
    if (commandedRpm > 0.0 && atSpeed(commandedRpm)) {
      if (Double.isNaN(atSpeedSinceSecs)) {
        atSpeedSinceSecs = atSpeedNow;
      }
    } else {
      atSpeedSinceSecs = Double.NaN;
    }
    Logger.recordOutput("Shooter/ReadyToFeed", readyToFeed());

    double now = Logger.getTimestamp() / 1.0e6;
    boolean struggling = commandedRpm > 0.0 && !atSpeed(commandedRpm);
    if (!struggling) {
      offTargetSinceSecs = Double.NaN;
    } else if (Double.isNaN(offTargetSinceSecs)) {
      offTargetSinceSecs = now;
    }

    Logger.recordOutput("Shooter/LeftErrorRpm", inputs.leftRpm - kTargetRpm);
    Logger.recordOutput("Shooter/RightErrorRpm", inputs.rightRpm - kTargetRpm);

    boolean stalling =
        inputs.leftStatorCurrentAmps > kStallCurrentAmps
            && inputs.rightStatorCurrentAmps > kStallCurrentAmps
            && Math.abs(inputs.leftRpm) < kStallRpm
            && Math.abs(inputs.rightRpm) < kStallRpm;
    Logger.recordOutput("Shooter/PossiblyFighting", stalling);
  }

  public Command setRpm(double rpm) {
    return setRpm(() -> rpm);
  }

  public Command setRpm(DoubleSupplier rpm) {
    return run(() -> {
          commandedRpm = rpm.getAsDouble();
          io.setRpm(commandedRpm);
        })
        .finallyDo(() -> commandedRpm = 0.0)
        .withName("ShooterSetRpm");
  }

  public boolean readyToFeed() {
    return !Double.isNaN(atSpeedSinceSecs)
        && Logger.getTimestamp() / 1.0e6 - atSpeedSinceSecs >= kSpinUpSettleSecs;
  }

  public boolean atSpeed(double rpm) {
    return Math.abs(inputs.leftRpm - rpm) <= kToleranceRpm
        && Math.abs(inputs.rightRpm - rpm) <= kToleranceRpm;
  }
}