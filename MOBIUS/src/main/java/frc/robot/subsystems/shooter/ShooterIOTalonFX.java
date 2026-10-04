// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left = new TalonFX(kLeftId, kCanBus);
  private final TalonFX right = new TalonFX(kRightId, kCanBus);

  private final StatusSignal<?> leftVelocity = left.getVelocity();
  private final StatusSignal<?> rightVelocity = right.getVelocity();
  private final StatusSignal<?> leftVolts = left.getMotorVoltage();
  private final StatusSignal<?> rightVolts = right.getMotorVoltage();
  private final StatusSignal<?> leftSupply = left.getSupplyVoltage();
  private final StatusSignal<?> rightSupply = right.getSupplyVoltage();
  private final StatusSignal<?> leftSupplyCurrent = left.getSupplyCurrent();
  private final StatusSignal<?> rightSupplyCurrent = right.getSupplyCurrent();
  private final StatusSignal<?> leftCurrent = left.getStatorCurrent();
  private final StatusSignal<?> rightCurrent = right.getStatorCurrent();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VelocityDutyCycle dutyRequest = new VelocityDutyCycle(0.0).withSlot(1);
  private final VelocityTorqueCurrentFOC torqueRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1).withOverrideCoastDurNeutral(true);
  private final ShooterBangBang bangBang = new ShooterBangBang();
  private final NeutralOut neutralRequest = new NeutralOut();

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = kStatorAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = kSupplyAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Never brake the flywheel. Above the setpoint it coasts down, which is free; braking burns
    // stored energy and gives the loop a powered restoring force on both sides to oscillate on.
    config.MotorOutput.PeakReverseDutyCycle = 0.0;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    if (kUseBangBang) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = kBangBangTorqueAmps;
      config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
      // Dedicated slot: voltage tuning must not overwrite bang-bang gains.
      // All other slot 1 gains remain zero so output is zero at/above target.
      config.Slot1.kP = kBangBangP;
    }
    config.Feedback.SensorToMechanismRatio = kGearRatio;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    config.MotorOutput.Inverted = PhoenixUtil.direction(kLeftInverted);
    left.getConfigurator().apply(config);

    config.MotorOutput.Inverted = PhoenixUtil.direction(kRightInverted);
    right.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        rightVelocity,
        leftVolts,
        rightVolts,
        leftCurrent,
        rightCurrent,
        leftSupply,
        rightSupply,
        leftSupplyCurrent,
        rightSupplyCurrent);
    left.optimizeBusUtilization();
    right.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftConnected =
        BaseStatusSignal.refreshAll(
                leftVelocity, leftVolts, leftCurrent, leftSupply, leftSupplyCurrent)
            .isOK();
    inputs.rightConnected =
        BaseStatusSignal.refreshAll(
                rightVelocity, rightVolts, rightCurrent, rightSupply, rightSupplyCurrent)
            .isOK();

    inputs.leftRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.rightRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftVolts.getValueAsDouble();
    inputs.rightAppliedVolts = rightVolts.getValueAsDouble();
    inputs.leftSupplyVolts = leftSupply.getValueAsDouble();
    inputs.rightSupplyVolts = rightSupply.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftCurrent.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightCurrent.getValueAsDouble();
  }

  @Override
  public void setRpm(double rpm) {
    if (!Double.isFinite(rpm) || rpm <= 0.0) {
      stop();
      return;
    }
    if (!kUseBangBang) {
      var request = velocityRequest.withVelocity(rpm / 60.0);
      left.setControl(request);
      right.setControl(request);
      Logger.recordOutput("Shooter/ControlMode", "VELOCITY_VOLTAGE");
      return;
    }
    boolean torque = bangBang.useTorque(
        rpm, leftVelocity.getValueAsDouble() * 60.0,
        rightVelocity.getValueAsDouble() * 60.0, Logger.getTimestamp() / 1.0e6);
    if (torque) {
      var request = torqueRequest.withVelocity(rpm / 60.0);
      left.setControl(request);
      right.setControl(request);
    } else {
      var request = dutyRequest.withVelocity(rpm / 60.0);
      left.setControl(request);
      right.setControl(request);
    }
    Logger.recordOutput("Shooter/ControlMode",
        torque ? "TORQUE_CURRENT_BANG_BANG" : "DUTY_CYCLE_BANG_BANG");
  }

  @Override
  public void setGains(double kP, double kS, double kV) {
    var slot = new Slot0Configs();
    slot.kS = kS;
    slot.kV = kV;
    slot.kA = kA;
    slot.kP = kP;
    slot.kI = kI;
    slot.kD = kD;
    left.getConfigurator().apply(slot);
    right.getConfigurator().apply(slot);
  }

  @Override
  public void stop() {
    bangBang.reset();
    Logger.recordOutput("Shooter/ControlMode", "COAST");
    left.setControl(neutralRequest);
    right.setControl(neutralRequest);
  }
}
