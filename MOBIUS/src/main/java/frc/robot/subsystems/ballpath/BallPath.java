// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballpath;

import static frc.robot.subsystems.ballpath.BallPathConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class BallPath extends SubsystemBase {
  private final BallPathIO io;
  private final BallPathIOInputsAutoLogged inputs = new BallPathIOInputsAutoLogged();

  private final double[] demand = new double[5];
  private final int[] jamLoops = new int[5];
  private final double[] clearUntilSecs = new double[5];
  private final boolean[] clearing = new boolean[5];

  private double pulseStartSecs = 0.0;
  private boolean loadStalled = false;
  private int loadStallLoops = 0;

  public BallPath(BallPathIO io) {
    this.io = io;
    setDefaultCommand(run(this::stopAll).withName("BallPathIdle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BallPath", inputs);
    updateJamDetection();
  }

  private void stopAll() {
    io.stop();
    java.util.Arrays.fill(demand, 0.0);
  }

  private void updateJamDetection() {
    double now = Logger.getTimestamp() / 1.0e6;
    for (int i = 0; i < demand.length; i++) {
      clearing[i] = now < clearUntilSecs[i];

      boolean stalled =
          !clearing[i]
              && Math.abs(demand[i]) > 0.01
              && inputs.statorCurrentAmps[i] > kJamCurrentAmps
              && Math.abs(inputs.velocityRotPerSec[i]) < kJamVelocityRotPerSec;

      jamLoops[i] = stalled ? jamLoops[i] + 1 : 0;
      if (jamLoops[i] >= kJamLoopsToTrigger) {
        clearUntilSecs[i] = now + kJamClearSecs;
        jamLoops[i] = 0;
      }
    }
    Logger.recordOutput("BallPath/Clearing", clearing);
  }

  private double withJamClear(int index, double percent) {
    demand[index] = percent;
    return clearing[index] && Math.abs(percent) > 0.01 ? -percent : percent;
  }

  public Command loadIndexer() {
    return run(() -> {
          java.util.Arrays.fill(demand, 0.0);

          if (!loadStalled) {
            boolean loaded = inputs.statorCurrentAmps[0] > kIntakeLoadStallAmps;
            loadStallLoops = loaded ? loadStallLoops + 1 : 0;
            if (loadStallLoops >= kIntakeLoadStallLoops) {
              loadStalled = true;
            }
          }

          io.setIndexer(loadStalled ? 0.0 : pulsedLoadPercent());
          io.setVerticalRoller(0.0);
          io.setSingulatorTop(0.0);
          io.setSingulatorBottom(0.0);
          io.setFeedRotPerSec(0.0);
          Logger.recordOutput("BallPath/LoadStalled", loadStalled);
        })
        .beforeStarting(
            () -> {
              loadStalled = false;
              loadStallLoops = 0;
              pulseStartSecs = Logger.getTimestamp() / 1.0e6;
            })
        .withName("BallPathLoad");
  }

  private double pulsed(double high, double low, double periodSecs) {
    double elapsed = Logger.getTimestamp() / 1.0e6 - pulseStartSecs;
    boolean fastPhase = ((long) Math.floor(elapsed / periodSecs)) % 2 == 0;
    return fastPhase ? high : low;
  }

  private double pulsedLoadPercent() {
    return pulsed(kIntakeLoadHighPercent, kIntakeLoadLowPercent, kIntakeLoadPulsePeriodSecs);
  }

  private double pulsedIndexerPercent() {
    return pulsed(kIndexerHighPercent, kIndexerLowPercent, kIndexerPulsePeriodSecs);
  }

  public Command runAll() {
    return runStages(
            this::pulsedIndexerPercent,
            kVerticalRollerPercent,
            kSingulatorTopPercent,
            kSingulatorBottomPercent,
            () -> kFeedRotPerSec)
        .beforeStarting(() -> pulseStartSecs = Logger.getTimestamp() / 1.0e6)
        .withName("BallPathRunAll");
  }

  public Command runIndexer() {
    return runStages(this::pulsedIndexerPercent, 0.0, 0.0, 0.0, () -> 0.0)
        .beforeStarting(() -> pulseStartSecs = Logger.getTimestamp() / 1.0e6)
        .withName("BallPathIndexerOnly");
  }

  public Command runSingulator() {
    return runStages(() -> 0.0, 0.0, kSingulatorTopPercent, kSingulatorBottomPercent, () -> 0.0)
        .withName("BallPathSingulatorOnly");
  }

  private Command runStages(
      DoubleSupplier indexer,
      double verticalRoller,
      double singulatorTop,
      double singulatorBottom,
      DoubleSupplier feed) {
    return run(() -> {
          io.setIndexer(withJamClear(0, indexer.getAsDouble()));
          io.setVerticalRoller(withJamClear(1, verticalRoller));
          io.setSingulatorTop(withJamClear(2, singulatorTop));
          io.setSingulatorBottom(withJamClear(3, singulatorBottom));
          io.setFeedRotPerSec(withJamClear(4, feed.getAsDouble()));
          Logger.recordOutput("BallPath/IndexerDemand", indexer.getAsDouble());
        });
  }
}
