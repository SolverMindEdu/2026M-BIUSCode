// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballpath;

import static frc.robot.subsystems.ballpath.BallPathConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.PhoenixUtil;

public class BallPathIOTalonFX implements BallPathIO {
  private final TalonFX indexer = PhoenixUtil.configureRoller(kIndexerId, kCanBus, kIndexerInverted, kStatorAmps, kSupplyAmps);
  private final TalonFX vertical = PhoenixUtil.configureRoller(kVerticalRollerId, kCanBus, kVerticalRollerInverted, kStatorAmps, kSupplyAmps);
  private final TalonFX singulatorTop = PhoenixUtil.configureRoller(kSingulatorTopId, kCanBus, kSingulatorTopInverted, kStatorAmps, kSupplyAmps);
  private final TalonFX singulatorBottom = PhoenixUtil.configureRoller(kSingulatorBottomId, kCanBus, kSingulatorBottomInverted, kStatorAmps, kSupplyAmps);
  private final TalonFX feed = PhoenixUtil.configureRoller(kFeedId, kCanBus, kFeedInverted, kStatorAmps, kSupplyAmps);

  private final TalonFX[] all = {indexer, vertical, singulatorTop, singulatorBottom, feed};

  private final StatusSignal<?>[] volts = new StatusSignal<?>[5];
  private final StatusSignal<?>[] current = new StatusSignal<?>[5];
  private final StatusSignal<?>[] velocity = new StatusSignal<?>[5];

  private final VoltageOut request = new VoltageOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  // Voltage rather than duty cycle: a duty cycle is a fraction of whatever the battery happens to
  // be, so every roller slows down exactly when the shooter and drive are pulling hardest. The
  // constants stay fractions of 12 V so the numbers mean the same thing they always did.
  private static final double kNominalVolts = 12.0;

  public BallPathIOTalonFX() {
    for (int i = 0; i < all.length; i++) {
      volts[i] = all[i].getMotorVoltage();
      current[i] = all[i].getStatorCurrent();
      velocity[i] = all[i].getVelocity();
      PhoenixUtil.publishRollerSignals(all[i], volts[i], current[i], velocity[i]);
    }

    var feedGains = new Slot0Configs();
    feedGains.kS = kFeedKs;
    feedGains.kV = kFeedKv;
    feedGains.kP = kFeedKp;
    feed.getConfigurator().apply(feedGains);
  }

  @Override
  public void updateInputs(BallPathIOInputs inputs) {
    for (int i = 0; i < all.length; i++) {
      inputs.connected[i] = BaseStatusSignal.refreshAll(volts[i], current[i], velocity[i]).isOK();
      inputs.appliedVolts[i] = volts[i].getValueAsDouble();
      inputs.statorCurrentAmps[i] = current[i].getValueAsDouble();
      inputs.velocityRotPerSec[i] = velocity[i].getValueAsDouble();
    }
  }

  @Override
  public void setIndexer(double percent) {
    indexer.setControl(request.withOutput(percent * kNominalVolts));
  }

  @Override
  public void setVerticalRoller(double percent) {
    vertical.setControl(request.withOutput(percent * kNominalVolts));
  }

  @Override
  public void setSingulatorTop(double percent) {
    singulatorTop.setControl(request.withOutput(percent * kNominalVolts));
  }

  @Override
  public void setSingulatorBottom(double percent) {
    singulatorBottom.setControl(request.withOutput(percent * kNominalVolts));
  }

  @Override
  public void setFeedRotPerSec(double rotPerSec) {
    feed.setControl(
        rotPerSec == 0.0 ? request.withOutput(0.0) : velocityRequest.withVelocity(rotPerSec));
  }

  @Override
  public void stop() {
    for (TalonFX motor : all) {
      motor.setControl(request.withOutput(0.0));
    }
  }
}
