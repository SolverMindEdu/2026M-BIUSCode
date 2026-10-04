// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double commandedDegrees = 0.0;
  private int limitLoops = 0;
  private boolean atTravelLimit = false;

  public Hood(HoodIO io) {
    this.io = io;
    setDefaultCommand(setAngle(kStowDegrees).withName("HoodStow"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    updateTravelLimit();
    Logger.recordOutput("Hood/CommandedDegrees", commandedDegrees);
    Logger.recordOutput("Hood/ErrorDegrees", commandedDegrees - inputs.positionDegrees);
    Logger.recordOutput("Hood/AtGoal", atGoal(commandedDegrees));
    Logger.recordOutput("Hood/TravelMeasured", kTravelMeasured);
    Logger.recordOutput("Hood/AtTravelLimit", atTravelLimit);
  }

  /**
   * A target past the end of the travel is as reached as it will ever be, whether the soft limit
   * cut the output or the hood is resting on its stop; without this the shot waits forever.
   */
  private void updateTravelLimit() {
    boolean reaching = commandedDegrees - inputs.positionDegrees > kToleranceDegrees;
    boolean stopped = Math.abs(inputs.velocityDegreesPerSec) < kStoppedDegPerSec;
    boolean outOfTravel =
        inputs.positionDegrees >= kMaxDegrees - kLimitMarginDegrees - kToleranceDegrees
            || inputs.statorCurrentAmps > kStopCurrentAmps;
    limitLoops = reaching && stopped && outOfTravel ? limitLoops + 1 : 0;
    if (limitLoops >= kTravelLimitLoops) {
      atTravelLimit = true;
      limitLoops = kTravelLimitLoops;
    } else if (!reaching) {
      atTravelLimit = false;
    }
  }

  public Command setAngle(double degrees) {
    return setAngle(() -> degrees);
  }

  public Command setAngle(DoubleSupplier degreesSupplier) {
    return run(() -> {
          commandedDegrees = degreesSupplier.getAsDouble();
          if (kTravelMeasured) {
            io.setAngle(commandedDegrees);
          } else {
            io.stop();
          }
        })
        .withName("HoodSetAngle");
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public boolean atGoal(double degrees) {
    return Math.abs(inputs.positionDegrees - degrees) <= kToleranceDegrees
        || (atTravelLimit && degrees > inputs.positionDegrees);
  }
  /** What the battery pays for this mechanism, for PowerMonitor. */
  public double supplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

}
