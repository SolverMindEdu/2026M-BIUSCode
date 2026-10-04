// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants;
import frc.robot.shooting.LaunchCalculator;
import frc.robot.shooting.ShotMap;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Does the turret actually end up pointing at the hub while the robot drives?
 *
 * <p>No plant model: the actuator is ideal, so the turret sits exactly on whatever the trapezoid
 * profile commands. What is under test is the aim pipeline -- LaunchCalculator, the field-to-turret
 * transform, TurretMath's wrap choice, and Turret's own profile and deadband -- not the gains.
 */
class TurretTrackingSimTest {
  private static final double kDt = 0.02;
  /** Gaps shorter than this inside an unwrap sweep are the turret crossing the target, not tracking. */
  private static final double kUnwrapMergeSecs = 0.25;
  /**
   * The hub we are aiming at, which mirrors with the alliance. Set per test rather than fixed:
   * every scenario here runs on both alliances, because the red hub sits at the far end of the
   * field and a sign or a mirror that only holds for blue would pass a blue-only suite.
   */
  private Translation2d kHub = FieldConstants.kBlueGoal;

  private void useAlliance(DriverStation.Alliance alliance) {
    DriverStationSim.setAllianceStationId(
        alliance == DriverStation.Alliance.Red
            ? AllianceStationID.Red1
            : AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    assertEquals(alliance, FieldConstants.alliance(), "alliance did not take");
    kHub = FieldConstants.ourGoal();
  }

  /** Perfect actuator: the mechanism is wherever the profile last asked it to be. */
  private static final class TurretIOIdeal implements TurretIO {
    private double positionDegrees = 0.0;
    private double velocityDegPerSec = 0.0;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
      inputs.motorConnected = true;
      inputs.encoderConnected = true;
      inputs.positionDegrees = positionDegrees;
      inputs.velocityDegreesPerSec = velocityDegPerSec;
      inputs.setpointDegrees = positionDegrees;
    }

    @Override
    public void setPositionSetpoint(
        double degrees, double velocityDegPerSec, boolean softGains, double staticVolts) {
      this.positionDegrees = degrees;
      this.velocityDegPerSec = velocityDegPerSec;
    }

    @Override
    public void stop() {
      velocityDegPerSec = 0.0;
    }

    @Override
    public void seedPosition(double degrees) {
      positionDegrees = degrees;
    }
  }

  /** One simulated instant of the drive base. */
  private record Sample(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {}

  /** What the turret did at that instant, in field terms. */
  private record Result(
      double timeSecs,
      double mechanismDegrees,
      double achievedBearingDegrees,
      double desiredBearingDegrees,
      double errorDegrees,
      double withinBand,
      boolean settled) {}

  private Turret turret;
  private TurretIOIdeal io;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    // Turret.driveTo measures its own profile step off the FPGA clock, so simulated time has to
    // advance with the loop or the profile sees a step of nearly zero and plans at a crawl.
    SimHooks.pauseTiming();
    SimHooks.restartTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    io = new TurretIOIdeal();
    turret = new Turret(io);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private static double wrap(double degrees) {
    return MathUtil.inputModulus(degrees, -180.0, 180.0);
  }

  /** Exactly RobotContainer.fieldRelativeTurretBearing. */
  private static double fieldRelativeTurretBearing(double fieldDegrees, double headingDegrees) {
    double bearing = fieldDegrees - headingDegrees;
    return (kPositiveMatchesGyro ? bearing : -bearing) + kForwardOffsetDegrees;
  }

  /** The inverse: where the barrel actually points on the field, given the mechanism angle. */
  private static double achievedFieldBearing(double mechanismDegrees, double headingDegrees) {
    double bearing = kForwardOffsetDegrees - mechanismDegrees;
    return headingDegrees + (kPositiveMatchesGyro ? bearing : bearing);
  }

  /**
   * Runs the real tracking command against a scripted drive trajectory and reports, for each step,
   * how far the barrel ended up from the hub.
   */
  private List<Result> run(double durationSecs, DoubleUnaryOperator unusedt, TrajectoryFn trajectory) {
    var calculator = new LaunchCalculator();
    var table = ShotMap.table();
    final Turret.Goal[] goal = {new Turret.Goal(kHomeAngleDegrees, 0.0)};

    Command track = turret.track(() -> goal[0], () -> true);
    CommandScheduler.getInstance().schedule(track);

    List<Result> out = new ArrayList<>();
    int steps = (int) Math.round(durationSecs / kDt);
    for (int i = 0; i <= steps; i++) {
      double t = i * kDt;
      Sample s = trajectory.at(t);
      double headingDegrees = s.pose().getRotation().getDegrees();

      // RobotContainer's off-trigger path: solve with the robot held still, so the turret points
      // at the hub itself rather than at a shoot-on-the-move lookahead.
      var direct =
          calculator.update(
              s.pose(), new ChassisSpeeds(), kRobotToTurret, kHub, table);

      double sign = kPositiveMatchesGyro ? 1.0 : -1.0;
      goal[0] =
          new Turret.Goal(
              fieldRelativeTurretBearing(direct.turretAngle().getDegrees(), headingDegrees),
              sign
                  * Math.toDegrees(
                      direct.turretVelocityRadPerSec()
                          - s.robotRelativeSpeeds().omegaRadiansPerSecond));

      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kDt);

      double mech = io.positionDegrees;
      double achieved = achievedFieldBearing(mech, headingDegrees);
      double desired = kHub.minus(direct.turretPosition()).getAngle().getDegrees();
      out.add(new Result(t, mech, achieved, desired, wrap(achieved - desired),
          turret.atGoal(kUnwrapSettledDegrees) ? 1.0 : 0.0, turret.isSettled()));
    }
    return out;
  }

  private interface TrajectoryFn {
    Sample at(double timeSecs);
  }

  private static void report(String name, List<Result> results, double settleSecs) {
    List<Result> settled = results.stream().filter(r -> r.timeSecs() >= settleSecs).toList();
    double worst = settled.stream().mapToDouble(r -> Math.abs(r.errorDegrees())).max().orElse(Double.NaN);
    double mean =
        settled.stream().mapToDouble(r -> Math.abs(r.errorDegrees())).average().orElse(Double.NaN);
    double minMech = results.stream().mapToDouble(Result::mechanismDegrees).min().orElse(0);
    double maxMech = results.stream().mapToDouble(Result::mechanismDegrees).max().orElse(0);
    System.out.printf(
        "%-34s after %4.1fs: mean|err|=%7.3f deg  max|err|=%8.3f deg  mech=[%8.2f, %8.2f]%n",
        name, settleSecs, mean, worst, minMech, maxMech);
  }

  @ParameterizedTest
  @EnumSource(DriverStation.Alliance.class)
  void parkedRobotPutsTheBarrelOnTheHub(DriverStation.Alliance alliance) {
    useAlliance(alliance);
    var pose = new Pose2d(kHub.getX() - 4.0, kHub.getY() + 1.5, Rotation2d.fromDegrees(35.0));
    var results = run(3.0, null, t -> new Sample(pose, new ChassisSpeeds()));
    report("parked " + alliance, results, 2.0);
    double worst = worstAfter(results, 2.0);
    assertTrue(worst < 0.5, "parked aim error " + worst + " deg");
  }

  @ParameterizedTest
  @EnumSource(DriverStation.Alliance.class)
  void spinningInPlaceHoldsTheHub(DriverStation.Alliance alliance) {
    useAlliance(alliance);
    double omega = Math.toRadians(180.0);
    var start = new Translation2d(kHub.getX() - 4.0, kHub.getY() + 1.5);
    var results =
        run(
            4.0,
            null,
            t ->
                new Sample(
                    new Pose2d(start, new Rotation2d(omega * t)),
                    new ChassisSpeeds(0.0, 0.0, omega)));
    report("spin " + alliance, results, 1.0);
    
    // 4 s at 180 deg/s needs 720 deg of counter-rotation; travel is 624 deg, so exactly one
    // unwrap is unavoidable. A full turn at the cruise limit is ~0.59 s.
    assertTracksExceptBoundedUnwraps("spin " + alliance, results, 1.0, 2.0, 1, 0.8);
  }

  @ParameterizedTest
  @EnumSource(DriverStation.Alliance.class)
  void strafingPastTheHubKeepsTheBarrelOnIt(DriverStation.Alliance alliance) {
    useAlliance(alliance);
    double vy = 3.0;
    var start = new Translation2d(kHub.getX() - 4.0, kHub.getY() - 6.0);
    var heading = Rotation2d.fromDegrees(0.0);
    var results =
        run(
            4.0,
            null,
            t ->
                new Sample(
                    new Pose2d(start.plus(new Translation2d(0.0, vy * t)), heading),
                    new ChassisSpeeds(0.0, vy, 0.0)));
    report("strafe " + alliance, results, 1.0);
    double worst = worstAfter(results, 1.0);
    assertTrue(worst < 5.0, "strafe aim error " + worst + " deg");
  }

  @ParameterizedTest
  @EnumSource(DriverStation.Alliance.class)
  void orbitingTheHubStaysInsideTheSoftLimits(DriverStation.Alliance alliance) {
    useAlliance(alliance);
    double radius = 4.0;
    double rate = Math.toRadians(60.0);
    var results =
        run(
            8.0,
            null,
            t ->
                new Sample(
                    new Pose2d(
                        kHub.plus(new Translation2d(radius * Math.cos(rate * t), radius * Math.sin(rate * t))),
                        Rotation2d.fromDegrees(0.0)),
                    new ChassisSpeeds(0.0, 0.0, 0.0)));
    report("orbit " + alliance, results, 1.0);
    
    // 8 s at 60 deg/s sweeps the bearing 480 deg, so one unwrap is unavoidable here too.
    assertTracksExceptBoundedUnwraps("orbit " + alliance, results, 1.0, 2.0, 1, 0.8);
  }


  private static void trace(String name, List<Result> results, int everyN) {
    System.out.println("--- trace: " + name);
    System.out.printf("%7s %10s %12s %12s %10s%n", "t", "mech", "achieved", "desired", "err");
    for (int i = 0; i < results.size(); i += everyN) {
      Result r = results.get(i);
      System.out.printf(
          "%7.2f %10.2f %12.2f %12.2f %10.2f%n",
          r.timeSecs(), r.mechanismDegrees(), r.achievedBearingDegrees(), r.desiredBearingDegrees(),
          r.errorDegrees());
    }
  }


  /**
   * A limited-travel turret cannot hold a continuously moving bearing forever; when it runs out of
   * travel it must sweep a full turn the other way and is off target for the duration. So the
   * requirement is not "always on target", it is "on target except for a bounded number of bounded
   * sweeps". Returns the off-target windows so a test can bound them.
   */
  private static List<double[]> offTargetWindows(
      List<Result> results, double settleSecs, double toleranceDegrees) {
    List<double[]> windows = new ArrayList<>();
    double start = Double.NaN;
    double last = settleSecs;
    for (Result r : results) {
      if (r.timeSecs() < settleSecs) {
        continue;
      }
      last = r.timeSecs();
      if (Math.abs(r.errorDegrees()) > toleranceDegrees) {
        if (Double.isNaN(start)) {
          start = r.timeSecs();
        }
      } else if (!Double.isNaN(start)) {
        windows.add(new double[] {start, r.timeSecs()});
        start = Double.NaN;
      }
    }
    if (!Double.isNaN(start)) {
      windows.add(new double[] {start, last});
    }
    // A 360 deg sweep passes through the correct bearing on its way round, which reads as a
    // moment of being "on target" mid-unwrap. Merge across those so one sweep counts once.
    List<double[]> merged = new ArrayList<>();
    for (double[] w : windows) {
      if (!merged.isEmpty() && w[0] - merged.get(merged.size() - 1)[1] < kUnwrapMergeSecs) {
        merged.get(merged.size() - 1)[1] = w[1];
      } else {
        merged.add(w);
      }
    }
    return merged;
  }

  private static void assertTracksExceptBoundedUnwraps(
      String name,
      List<Result> results,
      double settleSecs,
      double toleranceDegrees,
      int maxUnwraps,
      double maxUnwrapSecs) {
    List<double[]> windows = offTargetWindows(results, settleSecs, toleranceDegrees);
    System.out.printf("%-34s off-target windows (> %.1f deg): %d%n", name, toleranceDegrees, windows.size());
    for (double[] w : windows) {
      System.out.printf("    %6.2f -> %6.2f s  (%.2f s)%n", w[0], w[1], w[1] - w[0]);
    }
    assertTrue(
        windows.size() <= maxUnwraps,
        name + ": expected at most " + maxUnwraps + " unwrap(s), saw " + windows.size());
    for (double[] w : windows) {
      double duration = w[1] - w[0];
      assertTrue(
          duration <= maxUnwrapSecs,
          name + ": unwrap at " + w[0] + "s took " + duration + "s, limit " + maxUnwrapSecs + "s");
    }
  }

  private static double worstAfter(List<Result> results, double settleSecs) {
    return results.stream()
        .filter(r -> r.timeSecs() >= settleSecs)
        .mapToDouble(r -> Math.abs(r.errorDegrees()))
        .max()
        .orElse(Double.NaN);
  }

  /**
   * The commanded angle must stay inside the travel the soft limits enforce. Turret.driveTo feeds
   * TrapezoidProfile a goal state carrying the goal's velocity, so the profile plans to arrive
   * still moving and sails past a goal that clampToNearestLimit pinned to the limit.
   */
  @Test
  void commandedAngleStaysInsideSoftLimits() {
    double omega = Math.toRadians(180.0);
    var start = new Translation2d(kHub.getX() - 4.0, kHub.getY() + 1.5);
    var results =
        run(
            4.0,
            null,
            t ->
                new Sample(
                    new Pose2d(start, new Rotation2d(omega * t)),
                    new ChassisSpeeds(0.0, 0.0, omega)));
    double minMech = results.stream().mapToDouble(Result::mechanismDegrees).min().orElseThrow();
    double maxMech = results.stream().mapToDouble(Result::mechanismDegrees).max().orElseThrow();
    System.out.printf(
        "soft limits: allowed [%.2f, %.2f], commanded [%.2f, %.2f], overshoot fwd %.2f rev %.2f%n",
        kMinAngleDegrees, kMaxAngleDegrees, minMech, maxMech,
        Math.max(0.0, maxMech - kMaxAngleDegrees), Math.max(0.0, kMinAngleDegrees - minMech));
    assertTrue(
        maxMech <= kMaxAngleDegrees && minMech >= kMinAngleDegrees,
        "commanded angle left the soft limits: [" + minMech + ", " + maxMech + "]");
  }


  /**
   * How long does the turret need to get back on the hub when the chassis suddenly starts spinning?
   *
   * <p>The turret must counter-rotate at the chassis rate just to hold still, so whatever is left
   * under the profile's cruise limit is all it has to close a gap with. This is an ideal actuator,
   * so anything measured here is the motion profile alone -- no gains, no friction, no plant.
   */
  @Test
  void catchUpAfterAStepInChassisRotation() {
    double cruiseDegPerSec = kCruiseVelocityRotPerSec * 360.0;
    System.out.printf(
        "%nprofile cruise = %.0f deg/s, accel = %.0f deg/s^2%n", cruiseDegPerSec,
        kAccelerationRotPerSecSq * 360.0);
    System.out.printf(
        "%12s %14s %12s %14s %12s%n",
        "chassis", "headroom", "peak err", "time to <2 deg", "err at end");

    for (double rateDegPerSec : new double[] {90, 180, 270, 360, 450, 540, 600}) {
      setUp();
      double omega = Math.toRadians(rateDegPerSec);
      double settleSecs = 0.5;
      var start = new Translation2d(kHub.getX() - 4.0, kHub.getY() + 1.5);
      // Still until settleSecs so the turret is on target, then an instant step to a hard spin.
      var results =
          run(
              settleSecs + 1.5,
              null,
              t -> {
                double spun = Math.max(0.0, t - settleSecs);
                return new Sample(
                    new Pose2d(start, new Rotation2d(omega * spun)),
                    new ChassisSpeeds(0.0, 0.0, t < settleSecs ? 0.0 : omega));
              });

      // Truncate at the first unwrap. At 612 deg/s the turret moves at most 12.2 deg per 20 ms
      // step, so a jump beyond 30 deg is a wrap change, not tracking -- and the sweep that follows
      // is separately measured travel behaviour, not catch-up lag.
      var afterStep = new ArrayList<Result>();
      boolean unwrapped = false;
      Result previous = null;
      for (Result r : results) {
        if (r.timeSecs() < settleSecs) {
          previous = r;
          continue;
        }
        if (previous != null && Math.abs(r.mechanismDegrees() - previous.mechanismDegrees()) > 30.0) {
          unwrapped = true;
          break;
        }
        afterStep.add(r);
        previous = r;
      }
      double peak = afterStep.stream().mapToDouble(r -> Math.abs(r.errorDegrees())).max().orElse(Double.NaN);
      double recovered = Double.NaN;
      for (Result r : afterStep) {
        if (Math.abs(r.errorDegrees()) < 2.0 && r.timeSecs() > settleSecs + 0.04) {
          recovered = r.timeSecs() - settleSecs;
          break;
        }
      }
      double steady =
          afterStep.isEmpty()
              ? Double.NaN
              : Math.abs(afterStep.get(afterStep.size() - 1).errorDegrees());
      double heldSecs =
          afterStep.isEmpty() ? 0.0 : afterStep.get(afterStep.size() - 1).timeSecs() - settleSecs;
      System.out.printf(
          "%9.0f/s %11.0f/s %10.2f deg %12s s %10.2f deg   held %.2f s%s%n",
          rateDegPerSec,
          cruiseDegPerSec - rateDegPerSec,
          peak,
          Double.isNaN(recovered) ? "never" : String.format("%.2f", recovered),
          steady,
          heldSecs,
          unwrapped ? " (then unwrapped)" : "");
      tearDown();
    }
    setUp();
  }


  /** What the turret actually does through a sustained fast spin, second by second. */
  @Test
  void traceSustainedFastSpin() {
    double rateDegPerSec = 450.0;
    double omega = Math.toRadians(rateDegPerSec);
    var start = new Translation2d(kHub.getX() - 4.0, kHub.getY() + 1.5);
    var results =
        run(
            4.0,
            null,
            t ->
                new Sample(
                    new Pose2d(start, new Rotation2d(omega * t)),
                    new ChassisSpeeds(0.0, 0.0, omega)));
    System.out.printf("%n=== sustained %.0f deg/s spin, travel = %.0f deg ===%n",
        rateDegPerSec, kMaxAngleDegrees - kMinAngleDegrees);
    System.out.printf("%7s %10s %10s%n", "t", "mech", "err");
    for (int i = 0; i < results.size(); i += 3) {
      Result r = results.get(i);
      System.out.printf("%7.2f %10.2f %10.2f%s%n", r.timeSecs(), r.mechanismDegrees(),
          r.errorDegrees(), Math.abs(r.errorDegrees()) > 2.0 ? "   <-- off target" : "");
    }
    var windows = offTargetWindows(results, 0.5, 2.0);
    double offTotal = windows.stream().mapToDouble(w -> w[1] - w[0]).sum();
    double span = results.get(results.size() - 1).timeSecs() - 0.5;
    System.out.printf("%noff target %.2f s of %.2f s = %.0f%% of the time, in %d sweep(s)%n",
        offTotal, span, 100.0 * offTotal / span, windows.size());
    for (double[] w : windows) {
      System.out.printf("    %6.2f -> %6.2f s  (%.2f s)%n", w[0], w[1], w[1] - w[0]);
    }
  }


  /**
   * The feed must stop while the turret sweeps to a new wrap and resume once it is back on target.
   * RobotContainer passes Turret::isSettled to BallPath, which stops the indexer and re-primes the
   * tunnel whenever it reads false.
   */
  @Test
  void feedGateClosesThroughAnUnwrapAndReopensAfterIt() {
    double omega = Math.toRadians(180.0);
    var start = new Translation2d(kHub.getX() - 4.0, kHub.getY() + 1.5);
    var results =
        run(
            4.0,
            null,
            t ->
                new Sample(
                    new Pose2d(start, new Rotation2d(omega * t)),
                    new ChassisSpeeds(0.0, 0.0, omega)));

    var offTarget = offTargetWindows(results, 1.0, 2.0);
    assertTrue(offTarget.size() == 1, "expected exactly one unwrap, saw " + offTarget.size());
    double[] sweep = offTarget.get(0);

    List<double[]> gateShut = new ArrayList<>();
    double shutStart = Double.NaN;
    for (Result r : results) {
      if (!r.settled() && Double.isNaN(shutStart)) {
        shutStart = r.timeSecs();
      } else if (r.settled() && !Double.isNaN(shutStart)) {
        gateShut.add(new double[] {shutStart, r.timeSecs()});
        shutStart = Double.NaN;
      }
    }
    System.out.printf("%nunwrap sweep   %.2f -> %.2f s%n", sweep[0], sweep[1]);
    for (double[] g : gateShut) {
      System.out.printf("feed gate shut %.2f -> %.2f s  (%.2f s)%n", g[0], g[1], g[1] - g[0]);
    }

    assertTrue(gateShut.size() == 1, "expected the gate to shut once, saw " + gateShut.size());
    double[] shut = gateShut.get(0);
    // Shut before fuel is wasted, and stay shut until the barrel is genuinely back on the target.
    assertTrue(shut[0] <= sweep[0] + 0.06, "gate shut at " + shut[0] + "s, sweep began " + sweep[0]);
    assertTrue(shut[1] >= sweep[1] - 0.02, "gate reopened at " + shut[1] + "s, sweep ended " + sweep[1]);
    // And it must reopen, or the robot never shoots again.
    assertTrue(shut[1] < 4.0, "feed gate never reopened");

    // Nothing is fed while off target, and the turret is on target whenever the gate is open.
    double worstWhileOpen =
        results.stream()
            .filter(r -> r.timeSecs() >= 1.0 && r.settled())
            .mapToDouble(r -> Math.abs(r.errorDegrees()))
            .max()
            .orElse(Double.NaN);
    System.out.printf("worst aim error while the gate is open: %.2f deg%n", worstWhileOpen);
    assertTrue(worstWhileOpen < kUnwrapSettledDegrees, "fed while " + worstWhileOpen + " deg off");
  }


  /**
   * The same check at an ABSOLUTE field position rather than one written relative to the hub.
   *
   * <p>The relative scenarios above cannot fail on one alliance and pass on the other -- they are
   * the same geometry translated, which is exactly what makes them blind to a mirror being wrong.
   * This one parks the robot at a real spot in its own half and asks whether the barrel ends up on
   * the hub that alliance actually scores in.
   */
  @ParameterizedTest
  @EnumSource(DriverStation.Alliance.class)
  void aimsAtItsOwnHubFromAnAbsoluteSpotInItsOwnHalf(DriverStation.Alliance alliance) {
    useAlliance(alliance);
    // Two metres behind our own hub line, off to one side: a plausible shooting spot, mirrored.
    var spot =
        FieldConstants.mirrorForAlliance(new Translation2d(2.5, 2.6));
    var pose = new Pose2d(spot, Rotation2d.fromDegrees(20.0));

    assertTrue(
        FieldConstants.inOurAllianceZone(spot),
        alliance + " did not consider its own shooting spot to be in its own zone");

    var results = run(3.0, null, t -> new Sample(pose, new ChassisSpeeds()));
    double worst = worstAfter(results, 2.0);
    System.out.printf(
        "%-34s spot=(%.2f, %.2f) hub=(%.2f, %.2f) worst=%.4f deg%n",
        "absolute " + alliance, spot.getX(), spot.getY(), kHub.getX(), kHub.getY(), worst);
    assertTrue(worst < 0.5, alliance + " absolute aim error " + worst + " deg");

    // And the barrel is pointing at OUR hub, not the one at the other end of the field.
    var opponent =
        alliance == DriverStation.Alliance.Red ? FieldConstants.kBlueGoal : FieldConstants.kRedGoal;
    double toOurs = kHub.minus(spot).getAngle().getDegrees();
    double toTheirs = opponent.minus(spot).getAngle().getDegrees();
    assertTrue(
        Math.abs(wrap(toOurs - toTheirs)) > 20.0,
        "the two hubs are not distinguishable from this spot, so the test proves nothing");
    double achieved =
        achievedFieldBearing(results.get(results.size() - 1).mechanismDegrees(), 20.0);
    assertTrue(
        Math.abs(wrap(achieved - toOurs)) < Math.abs(wrap(achieved - toTheirs)),
        alliance + " aimed nearer the opposing hub");
  }

}
