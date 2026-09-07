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
    setDefaultCommand(run(io::stop).withName("IntakeRollerIdle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);
  }

  public Command intake() {
    return run(() -> io.setPercent(kIntakePercent)).withName("IntakeRollerRun");
  }

  public Command shootAssist() {
    return run(() -> io.setPercent(kShootPercent)).withName("IntakeRollerShootAssist");
  }
}
