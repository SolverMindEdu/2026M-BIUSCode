// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballpath;

import static frc.robot.subsystems.ballpath.BallPathConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class BallPath extends SubsystemBase {
  private final BallPathIO io;
  private final BallPathIOInputsAutoLogged inputs = new BallPathIOInputsAutoLogged();

  /** Holds the indexer off until the tunnel has re-primed after an aim loss. */
  private double indexerReadySecs = 0.0;


  /** Indexer, tunnel and both singulators; the kicker has no entry and is never reversed. */
  private static final int kClearableStages = 4;

  private final double[] demand = new double[kClearableStages];
  private final int[] jamLoops = new int[kClearableStages];
  private final double[] clearUntilSecs = new double[kClearableStages];
  private final boolean[] clearing = new boolean[kClearableStages];

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
    for (int i = 0; i < kClearableStages; i++) {
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

  /** A short reverse once a stage has stalled; speeds are untouched either way. */
  private double withJamClear(int index, double percent) {
    demand[index] = percent;
    return clearing[index] && Math.abs(percent) > 0.01 ? -percent : percent;
  }

  /** Runs the tunnel and kicker while aim is ready; the kicker keeps spinning regardless. */
  public Command runAll(BooleanSupplier aimReady) {
    return feedWhen(aimReady, kIndexerPercent).withName("BallPathMovingShotFeed");
  }

  /** Everything except the indexer, so the tunnel reaches speed before any fuel is pushed in. */
  public Command primeStages(BooleanSupplier aimReady) {
    return feedWhen(aimReady, 0.0).withName("BallPathMovingShotPrimeStages");
  }

  private Command feedWhen(BooleanSupplier aimReady, double indexerPercent) {
    return run(() -> {
          double now = Logger.getTimestamp() / 1.0e6;
          boolean aimed = aimReady.getAsBoolean();
          if (!aimed) {
            // Every resume re-primes, so the tunnel is already turning before fuel arrives.
            indexerReadySecs = now + kStagePrimeSecs;
          }
          boolean feed = aimed && now >= indexerReadySecs;

          io.setIndexer(withJamClear(0, feed ? indexerPercent : 0.0));
          // Transport runs for the whole trigger hold; stopping it left the singulator having to
          // restart against a packed column, which is what jams it.
          io.setVerticalRoller(withJamClear(1, kVerticalRollerPercent));
          // Slowed, not stopped, while the indexer is held: see kSingulatorHoldScale.
          double singulator = kSingulatorTopPercent * (feed ? 1.0 : kSingulatorHoldScale);
          io.setSingulatorTop(withJamClear(2, singulator));
          io.setSingulatorBottom(withJamClear(3, singulator));
          // Never handed back mid-shot: while the trigger is held the kicker spins, full stop.
          io.setFeedRotPerSec(kFeedRotPerSec);

          Logger.recordOutput("BallPath/Aimed", aimed);
          Logger.recordOutput("BallPath/IndexerFeeding", feed);
          Logger.recordOutput("BallPath/IndexerPercent", feed ? indexerPercent : 0.0);
          Logger.recordOutput("BallPath/SingulatorPercent", singulator);
        });
  }

  /** Every stage backwards on the driver's command; nothing else ever runs in reverse. */
  public Command reverseAll() {
    return run(() -> {
          java.util.Arrays.fill(demand, 0.0);
          io.setIndexer(-kIndexerPercent);
          io.setVerticalRoller(-kVerticalRollerPercent);
          io.setSingulatorTop(-kSingulatorTopPercent);
          io.setSingulatorBottom(-kSingulatorTopPercent);
          io.setFeedRotPerSec(-kFeedRotPerSec);
        })
        .finallyDo(this::stopAll)
        .withName("BallPathReverseAll");
  }

  /** Only the kicker, which needs about 180 ms to reach speed; nothing moves a ball forward yet. */
  public Command prime() {
    return run(() -> {
          java.util.Arrays.fill(demand, 0.0);
          io.setIndexer(0.0);
          io.setVerticalRoller(0.0);
          io.setSingulatorTop(0.0);
          io.setSingulatorBottom(0.0);
          io.setFeedRotPerSec(kFeedRotPerSec);
        })
        .withName("BallPathPrime");
  }

  public boolean isConnected() {
    for (boolean stage : inputs.connected) {
      if (!stage) {
        return false;
      }
    }
    return true;
  }
  /** Every stage together, which is what the battery sees from this mechanism. */
  public double supplyCurrentAmps() {
    double total = 0.0;
    for (double amps : inputs.supplyCurrentAmps) {
      total += amps;
    }
    return total;
  }

}
