// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeRollerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();
  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);
  }

  public Command intake() {
    return run(() -> io.setPercent(kIntakePercent)).withName("IntakeRollerRun");
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  /** Rollers off, without giving up the subsystem. */
  public Command idle() {
    return run(io::stop).withName("IntakeRollerIdle");
  }

  public Command purge() {
    return run(() -> io.setPercent(-kPurgePercent)).withName("IntakeRollerPurge");
  }

  public Command shootAssist() {
    return run(() -> io.setPercent(kShootPercent)).beforeStarting(() -> io.setStatorLimit(kShootStatorAmps))
        .finallyDo(() -> io.setStatorLimit(kStatorAmps))
        .withName("IntakeRollerShootAssist");
  }
  /** What the battery pays for this mechanism, for PowerMonitor. */
  public double supplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

}
