// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.io.File;
import java.util.Arrays;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ballpath.BallPath;
import frc.robot.subsystems.ballpath.BallPathConstants;
import frc.robot.subsystems.ballpath.BallPathIO;
import frc.robot.subsystems.ballpath.BallPathIOTalonFX;
import frc.robot.FieldConstants.PassSide;
import frc.robot.shooting.PassMap;
import frc.robot.shooting.ShotMap;
import frc.robot.shooting.LaunchCalculator;
import frc.robot.util.PowerMonitor;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerConstants;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.intake.IntakeDeployIO;
import frc.robot.subsystems.intake.IntakeDeployIOTalonFX;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Vision vision;
  private final Turret turret;
  private final BallPath ballPath;
  private final Hood hood;
  private final Shooter shooter;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;

  private final CommandSwerveDrivetrain drive;

  /** Comes up light blue with the robot; states get added as we decide them. */
  private final Leds leds = new Leds();

  /** Null when the drivetrain is absent or PathPlanner has no GUI settings to configure from. */
  private LoggedDashboardChooser<Command> autoChooser;

  /** Null when the drivetrain is absent; Choreo routines are written against this. */
  private AutoFactory autoFactory;

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);

  private static final double kStickDeadband = 0.1;

  // 11.0, not 12.0: a healthy battery under load sits near 12 V, so the offsets flipped shot to shot.
  private static final double kShotLowBatteryThresholdVolts = 11.0;
  private static final double kShotLowBatteryHoodOffsetDegrees = 10.0;

  /** A sagging rail costs flywheel speed, so ask for more than the map says. */
  private static final double kShotLowBatteryRpmOffset = 30.0;

  /** Driver rumble starts below this battery voltage. */
  private static final double kRumbleStartVolts = 9.0;

  /** Rumble reaches full strength here, a margin above the roboRIO's 6.75 V brownout. */
  private static final double kRumbleFullVolts = 7.5;

  // 0.2 s smoothing, so one current spike from grabbing a ball does not buzz the controller.
  private final LinearFilter rumbleVoltageFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
  private double shotHoodOffsetDegrees = 0.0;
  /** Latched by right bumper during a shot; the slow stow never starts on its own. */
  private boolean shootStowRequested = false;

  /** True only once the shot has stopped waiting on aim and is actually putting fuel through. */
  private boolean feeding = false;

  private static final double kDriveSpeedScale = 0.70;
  private static final double kShootingDriveSpeedScale = 0.15;

  /** A pass does not need the accuracy a hub shot does, so it keeps more of the driver's speed. */
  private static final double kPassDriveSpeedScale = 0.45;
  private final PowerMonitor powerMonitor = new PowerMonitor();

  private final LaunchCalculator launchCalculator = new LaunchCalculator();

  /** Same solver with the robot held still, so off the trigger the turret points at the real hub. */
  private final LaunchCalculator trackCalculator = new LaunchCalculator();

  private boolean launching = false;
  private boolean prepping = false;
  private LaunchCalculator.Parameters launch;
  private Turret.Goal turretGoal = new Turret.Goal(TurretConstants.kHomeAngleDegrees, 0.0);
  private double shotHeadingDegrees;

  private static final double kDriveTurnScale = 1.0;


  /** Wheels in an X so the robot resists being pushed while the turret keeps aiming. */
  private final SwerveRequest.SwerveDriveBrake defenseRequest = new SwerveRequest.SwerveDriveBrake();

  private boolean defenseMode = false;
  private boolean defenseLocked = false;

  /** Defense mode only locks once the robot has nearly stopped, so modules never snap into an X at speed. */
  private static final double kDefenseLockSpeedMetersPerSec = 0.3;

  private static final double kDefenseLockTurnRadPerSec = 0.5;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0)
          .withRotationalDeadband(0.0)
          .withDriveRequestType(
              com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case REAL, SIM -> {
        vision =
            kVisionEnabled
                ? new Vision(
                    this::addVisionMeasurement,
                    new VisionIOLimelight(camera0Name, this::yawSupplier),
                    new VisionIOLimelight(camera1Name, this::yawSupplier),
                    new VisionIOLimelight(camera2Name, this::yawSupplier))
                : new Vision(
                    this::addVisionMeasurement,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {});
        turret = new Turret(new TurretIOTalonFX());
        ballPath = new BallPath(new BallPathIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        intakeDeploy = new IntakeDeploy(new IntakeDeployIOTalonFX());
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
        drive = isDriveConfigured() ? TunerConstants.createDrivetrain() : null;
      }

      case REPLAY -> {
        vision =
            new Vision(
                this::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        turret = new Turret(new TurretIO() {});
        ballPath = new BallPath(new BallPathIO() {});
        hood = new Hood(new HoodIO() {});
        shooter = new Shooter(new ShooterIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        drive = null;
      }

      default -> throw new IllegalStateException("Unhandled mode: " + Constants.kCurrentMode);
    }

    SmartDashboard.putData("Field", field);

    if (drive != null) {
      autoFactory = drive.createAutoFactory();
      registerAutoCommands(autoFactory);
      // PathPlanner discovers its own autos from deploy/pathplanner/autos; Choreo does not, so
      // its trajectories are added by hand below.
      autoChooser =
          new LoggedDashboardChooser<>(
              "Auto",
              drive.isAutoBuilderConfigured()
                  ? AutoBuilder.buildAutoChooser()
                  : new SendableChooser<>());
      addChoreoRoutines(autoChooser);
    }

    configureBindings();
  }

  private static boolean isDriveConfigured() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) > 0.0;
  }

  private Rotation2d yawSupplier() {
    return drive == null ? Rotation2d.kZero : drive.getState().Pose.getRotation();
  }

  private void addVisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    if (drive != null) {
      drive.addVisionMeasurement(pose, timestampSeconds, stdDevs);
    }
    Logger.recordOutput("Vision/PendingMeasurement/Pose", pose);
    Logger.recordOutput("Vision/PendingMeasurement/Timestamp", timestampSeconds);
  }

  private void configureBindings() {
    if (drive != null) {
      drive.setDefaultCommand(
          drive.applyRequest(
              () -> {
                Translation2d translation = shapedTranslation();
                double rotation = shapedAxis(-driverController.getRightX());
                boolean sticksIdle = translation.getNorm() == 0.0 && rotation == 0.0;
                // Lock once stopped, and stay locked through a shove until the driver moves a stick.
                if (!defenseMode || !sticksIdle) {
                  defenseLocked = false;
                } else if (!defenseLocked && robotNearlyStopped()) {
                  defenseLocked = true;
                }
                if (defenseLocked) {
                  return defenseRequest;
                }
                boolean shooting = driverController.getRightTriggerAxis() > 0.5;
                double aimingScale = passMode ? kPassDriveSpeedScale : kShootingDriveSpeedScale;
                double speedScale = shooting ? aimingScale : kDriveSpeedScale;
                double turnScale = shooting ? aimingScale : kDriveTurnScale;
                return driveRequest
                    .withVelocityX(translation.getX() * maxSpeed * speedScale)
                    .withVelocityY(translation.getY() * maxSpeed * speedScale)
                    .withRotationalRate(rotation * maxAngularRate * turnScale);
              }));
    }

    driverController.rightTrigger(0.5).whileTrue(shoot());

    // X toggles defense mode: drive normally, and the wheels X-lock whenever the sticks are released.
    driverController.x().onTrue(Commands.runOnce(() -> defenseMode = !defenseMode));

    // D-pad down runs the whole path and the intake backwards, to clear a jam.
    driverController
        .povDown()
        .whileTrue(Commands.parallel(ballPath.reverseAll(), intakeRoller.purge()));

    // Y toggles pass mode on and off.
    driverController.y().onTrue(Commands.runOnce(() -> passMode = !passMode).ignoringDisable(true));

    // The flywheel holds spin-up speed whenever nothing else is commanding it, so it is already
    // turning at the next shot; D-pad up switches that off when the battery needs the current.
    shooter.setDefaultCommand(
        shooter
            .setRpm(() -> flywheelIdle ? ShooterConstants.kIdleRpm : 0.0)
            .withName("ShooterIdle"));
    driverController
        .povUp()
        .onTrue(Commands.runOnce(() -> flywheelIdle = !flywheelIdle).ignoringDisable(true));

    // Intake modes own only the intake, so switching left trigger never restarts the shot.
    Trigger shootingButton = driverController.rightTrigger(0.5);
    Trigger leftTrigger = driverController.leftTrigger(0.5);
    shootingButton.and(leftTrigger).whileTrue(
        Commands.parallel(
            IntakeDeploy.isTravelMeasured() ? intakeDeploy.deploy() : intakeDeploy.jog(() -> 1.0),
            intakeRoller.intake()));
    shootingButton
        .and(driverController.rightBumper())
        .onTrue(Commands.runOnce(() -> shootStowRequested = true));
    shootingButton.and(leftTrigger.negate()).and(() -> shootStowRequested).whileTrue(
        Commands.parallel(intakeDeploy.shootStow(), intakeRoller.shootAssist()));

    Trigger intakeButton = leftTrigger.and(shootingButton.negate());
    // The very first deploy kicks the rollers backwards briefly to clear anything sitting in
    // them; every deploy after that goes straight to intaking.
    intakeButton.whileTrue(
        Commands.either(
            intakeRoller.intake(),
            intakeRoller
                .purge()
                .withTimeout(IntakeRollerConstants.kPurgeSecs)
                .andThen(Commands.runOnce(() -> rollersPurged = true))
                .andThen(intakeRoller.intake()),
            () -> rollersPurged));
    if (IntakeDeploy.isTravelMeasured()) {
      intakeButton.onTrue(intakeDeploy.deploy());
      // A pulls the intake home; otherwise it stays out after intaking until the next shot.
      driverController.a().onTrue(intakeDeploy.retract());
    } else {
      intakeButton.whileTrue(intakeDeploy.jog(() -> 1.0));
    }

    // Tracks continuously, auto included; PrepShoot and Shoot only change what it aims at.
    turret.setDefaultCommand(turret.track(() -> turretGoal, () -> true));

    // Recalls the turret to zero and then ends, which hands it back to the tracking default command.
    driverController.b().onTrue(turret.goToZero());
  }

  private boolean robotNearlyStopped() {
    var speeds = drive.getState().Speeds;
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            < kDefenseLockSpeedMetersPerSec
        && Math.abs(speeds.omegaRadiansPerSecond) < kDefenseLockTurnRadPerSec;
  }

  private Translation2d shapedTranslation() {
    double x = -driverController.getLeftY();
    double y = -driverController.getLeftX();
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), kStickDeadband);
    if (magnitude == 0.0) {
      return Translation2d.kZero;
    }
    return new Translation2d(magnitude * magnitude, new Rotation2d(x, y));
  }

  private static double shapedAxis(double value) {
    double deadbanded = MathUtil.applyDeadband(value, kStickDeadband);
    return Math.copySign(deadbanded * deadbanded, deadbanded);
  }

  public void logDriveState() {
    if (drive == null) {
      return;
    }
    var state = drive.getState();
    Logger.recordOutput("Drive/Pose", state.Pose);

    Logger.recordOutput("Drive/Pose3d", new Pose3d(state.Pose));
    Logger.recordOutput("Drive/HeadingDegrees", state.Pose.getRotation().getDegrees());
    Logger.recordOutput("Drive/Speeds", state.Speeds);

    // Always on, so the distance can be read while parked and not shooting -- that is the number
    // every shot map row is keyed to.
    currentShot();
  }

  /** One solve per loop, so turret, hood, and flywheel all use the same drive sample. */
  public void updateMovingShot() {
    if (drive == null) {
      return;
    }
    var state = drive.getState();
    turret.setVisionConfident(vision.hasRecentMultiTagVision());
    // Outside the scoring zone and off the trigger, an unwrap costs nothing, so take it early.
    turret.setRecentre(
        !launching && !FieldConstants.inOurAllianceZone(state.Pose.getTranslation()));
    shotHeadingDegrees = state.Pose.getRotation().getDegrees();
    // Which half we are in decides which shot we are taking: see passModeFor.
    boolean allianceKnown = FieldConstants.allianceKnown();
    boolean inAllianceZone = FieldConstants.inOurAllianceZone(state.Pose.getTranslation());
    passMode = passModeFor(allianceKnown, inAllianceZone, wasInAllianceZone, passMode);
    wasInAllianceZone = inAllianceZone;
    Logger.recordOutput("Shot/InAllianceZone", inAllianceZone);
    Logger.recordOutput("Shot/AllianceKnown", allianceKnown);
    shotTarget =
        passMode
            ? FieldConstants.passTarget(state.Pose.getTranslation(), passSide)
            : FieldConstants.ourGoal();
    var table = passMode ? PassMap.table() : ShotMap.table();
    // Both run every loop so each keeps its own rate filter and neither jumps when the trigger is
    // pulled: one leads the target by the flight time, the other just points at it.
    var led =
        launchCalculator.update(
            state.Pose, state.Speeds, TurretConstants.kRobotToTurret, shotTarget, table);
    var direct =
        trackCalculator.update(
            state.Pose, new ChassisSpeeds(), TurretConstants.kRobotToTurret, shotTarget, table);
    // Auto leads while it is only driving, so nothing swings when a shot starts mid-path. PrepShoot
    // is the exception: it marks the approach to a shooting spot, and there the lead is about to
    // collapse to nothing anyway, so pointing straight at the hub settles the turret early instead
    // of letting it chase a lookahead that is on its way out. Teleop points straight until the
    // trigger is pulled.
    boolean leading = launching || (DriverStation.isAutonomous() && !prepping);
    launch = leading ? led : direct;
    Logger.recordOutput("Shot/Leading", leading);
    Logger.recordOutput("Shot/Prepping", prepping);

    // Into the turret's frame; the robot's own spin comes off the rate here, as 6328 does in theirs.
    double sign = TurretConstants.kPositiveMatchesGyro ? 1.0 : -1.0;
    turretGoal =
        new Turret.Goal(
            fieldRelativeTurretBearing(launch.turretAngle().getDegrees()),
            sign
                * Math.toDegrees(
                    launch.turretVelocityRadPerSec() - state.Speeds.omegaRadiansPerSecond));

    boolean passingLeft = passMode && FieldConstants.isDriverLeft(shotTarget);
    Logger.recordOutput("Shot/PassingLeft", passingLeft);
    Logger.recordOutput("Shot/TrackingHub", !passMode);
    Logger.recordOutput("Shot/PassingRight", passMode && !passingLeft);
    Logger.recordOutput("Shot/PassSide", passSide.toString());
    Logger.recordOutput("Shot/PassMode", passMode);
    Logger.recordOutput("Shot/Target", shotTarget);
    Logger.recordOutput(
        "Shot/GoalDistanceMeters",
        FieldConstants.ourGoal().getDistance(state.Pose.getTranslation()));
    Logger.recordOutput("Shot/TurretPosition", launch.turretPosition());
    Logger.recordOutput("Shot/LookaheadPosition", launch.lookaheadPosition());
    Logger.recordOutput("Shot/LookaheadDistanceMeters", launch.distanceMeters());
    Logger.recordOutput("Shot/TimeOfFlightSecs", launch.timeOfFlightSecs());
    Logger.recordOutput("Shot/WithinTableRange", launch.withinTableRange());
    Logger.recordOutput("Shot/TurretFieldAngleDegrees", launch.turretAngle().getDegrees());
    Logger.recordOutput(
        "Shot/TurretFieldRateDegPerSec", Math.toDegrees(launch.turretVelocityRadPerSec()));
    field.setRobotPose(state.Pose);
    field.getObject("Target").setPose(new Pose2d(shotTarget, Rotation2d.kZero));
  }

  /**
   * Which shot the robot is set up for, decided by which half of the field it is in.
   *
   * <p>G407 only allows launching FUEL at our HUB while our bumpers are at least partly in our own
   * ALLIANCE ZONE, so out in the NEUTRAL ZONE the pass is the only legal shot there is. That is a
   * rule, not a preference, so out there pass mode is held on and the driver's toggle cannot turn
   * it off -- a mis-press would otherwise be a MAJOR FOUL. Coming back over the bump hands the
   * choice back: pass mode clears on the loop we re-enter the zone, and the toggle works normally
   * from then on, because passing from inside our own zone is perfectly legal.
   *
   * <p>None of that is safe to act on until the driver station has named an alliance, since the
   * zone test mirrors with it; until then the driver keeps whatever they chose.
   */
  static boolean passModeFor(
      boolean allianceKnown, boolean inAllianceZone, boolean wasInAllianceZone, boolean passMode) {
    if (!allianceKnown) {
      return passMode;
    }
    if (!inAllianceZone) {
      return true;
    }
    return wasInAllianceZone ? passMode : false;
  }

  private boolean shotAimReady() {
    boolean solutionReady = launch != null;
    boolean hoodReady = hood.atGoal(currentShot().hoodDegrees());
    boolean turretReady = turret.atGoal();
    Logger.recordOutput("Shot/FeedGate/SolutionReady", solutionReady);
    Logger.recordOutput("Shot/FeedGate/HoodReady", hoodReady);
    Logger.recordOutput("Shot/FeedGate/TurretReady", turretReady);
    Logger.recordOutput("Shot/FeedGate/FlywheelReady", shooter.readyToFeed());
    Logger.recordOutput(
        "Shot/FeedGate/InTableRange",
        launch != null && launch.withinTableRange());
    boolean aimReady = solutionReady && hoodReady && turretReady;
    boolean turretOnTarget = turret.atGoal(TurretConstants.kStartShotToleranceDegrees);
    Logger.recordOutput("Shot/FeedGate/TurretOnTarget", turretOnTarget);
    // Single flag for the driver dashboard: everything the first ball waits on.
    Logger.recordOutput(
        "Shot/ReadyToShoot", aimReady && turretOnTarget && shooter.readyToFeed());
    return aimReady;
  }

  private boolean passMode = false;
  private boolean wasInAllianceZone = false;
  private Translation2d shotTarget = FieldConstants.kBlueGoal;
  private FieldConstants.PassSide passSide = FieldConstants.PassSide.BOTH;

  private double shotRpmOffset = 0.0;

  // Toggle buttons rather than command buttons: they latch, so the button is its own indicator,
  // and unlike a Command widget they can be resized below 256 px.
  private final BooleanEntry[] passEntries = passEntries();
  private PassSide lastPublishedSide = null;

  // Elastic's Field widget reads a Field2d sendable, not a bare struct output.
  private final Field2d field = new Field2d();

  private boolean flywheelIdle = true;

  private boolean rollersPurged = false;

  private void snapshotShotVoltage() {
    // Called once per right-trigger press, before the shooting commands start.
    double voltage = RobotController.getBatteryVoltage();
    boolean lowBattery = voltage < kShotLowBatteryThresholdVolts;
    shotHoodOffsetDegrees = lowBattery ? kShotLowBatteryHoodOffsetDegrees : 0.0;
    shotRpmOffset = lowBattery ? kShotLowBatteryRpmOffset : 0.0;
    Logger.recordOutput("Shot/BatterySnapshotVolts", voltage);
    Logger.recordOutput("Shot/HoodOffsetDegrees", shotHoodOffsetDegrees);
    Logger.recordOutput("Shot/RpmOffset", shotRpmOffset);
  }

  private ShotMap.Shot currentShot() {
    double distance = launch == null ? 0.0 : launch.distanceMeters();
    ShotMap.Shot base =
        launch == null
            ? ShotMap.lookup(distance)
            : new ShotMap.Shot(launch.hoodDegrees(), launch.rpm());
    ShotMap.Shot shot =
        new ShotMap.Shot(
            base.hoodDegrees() + shotHoodOffsetDegrees, base.shooterRpm() + shotRpmOffset);
    Logger.recordOutput("Shot/DistanceMeters", distance);
    Logger.recordOutput("Shot/MapHoodDegrees", shot.hoodDegrees());
    Logger.recordOutput("Shot/MapShooterRpm", shot.shooterRpm());
    return shot;
  }

  private double fieldRelativeTurretBearing(double fieldDegrees) {
    double chassisHeadingDegrees =
        drive == null ? 0.0 : shotHeadingDegrees;
    Logger.recordOutput("Turret/ChassisHeadingDegrees", chassisHeadingDegrees);

    double bearing = fieldDegrees - chassisHeadingDegrees;
    return (TurretConstants.kPositiveMatchesGyro ? bearing : -bearing)
        + TurretConstants.kForwardOffsetDegrees;
  }

  /** Spin up and feed. Runs until cancelled. */
  private Command shoot() {
    return Commands.parallel(
            shooter.setRpm(() -> currentShot().shooterRpm()),
            hood.setAngle(() -> currentShot().hoodDegrees()),
            // An unwrap is the ONLY thing that holds fuel back. While the turret sweeps to a new
            // wrap the barrel points nowhere near the target, so those balls are simply thrown
            // away; Turret.isSettled carries its own kUnwrapTimeoutSecs backstop, so the sweep can
            // never hold the feed off indefinitely.
            //
            // Nothing else gates. The flywheel in particular does not: it leaves its tolerance band
            // on every ball, and BallPath re-primes the tunnel on each resume, so checking it per
            // ball cost about a third of a second every time and turned a burst into single shots.
            // Shooter/ReadyToFeed is still logged for tuning, it just does not stop the feeder.
            ballPath
                .primeStages(turret::isSettled)
                .withTimeout(BallPathConstants.kStagePrimeSecs)
                .andThen(Commands.runOnce(() -> feeding = true))
                .andThen(ballPath.runAll(turret::isSettled)))
        .beforeStarting(() -> {
          snapshotShotVoltage();
          launching = true;
        })
        .finallyDo(() -> {
          launching = false;
          shotHoodOffsetDegrees = 0.0;
          shotRpmOffset = 0.0;
          shootStowRequested = false;
          feeding = false;
        });
  }

  /** Shift clock and goal state, which FMS only reveals a few seconds into teleop. */
  public void logMatchState() {
    var state =
        MatchState.of(
            DriverStation.isAutonomous(),
            DriverStation.getMatchTime(),
            DriverStation.getGameSpecificMessage(),
            FieldConstants.alliance());
    Logger.recordOutput("Match/Phase", state.phase().toString());
    Logger.recordOutput("Match/ShiftNumber", state.shiftNumber());
    Logger.recordOutput("Match/ShiftSecsRemaining", state.shiftSecsRemaining());
    Logger.recordOutput("Match/SecsRemaining", state.matchSecsRemaining());
    Logger.recordOutput("Match/OurGoalActive", state.ourGoalActive());
    Logger.recordOutput("Match/ShiftDataValid", state.dataValid());
  }

  /**
   * Which pass side is armed, which is a selection rather than a live state: exactly one is true
   * from boot, defaulting to BOTH, whether or not the robot is passing right now.
   */
  public void logDashboardState() {
    // Never carry a wheel lock into the next enable, where the driver would find the robot won't move.
    if (DriverStation.isDisabled()) {
      defenseMode = false;
      defenseLocked = false;
    }
    vision.setDefenseMode(defenseLocked);
    Logger.recordOutput("Drive/DefenseMode", defenseMode);
    Logger.recordOutput("Drive/DefenseLocked", defenseLocked);
    updatePassSelection();
    updateBrownoutRumble();
    Logger.recordOutput("Shot/Feeding", feeding);
    // Nothing gates on this any more; it is kept so the driver can still see what the shot looks like.
    shotAimReady();

    Logger.recordOutput("Shooter/IdleEnabled", flywheelIdle);
    Logger.recordOutput("Status/Turret", turret.isConnected());
    Logger.recordOutput("Status/Hood", hood.isConnected());
    Logger.recordOutput("Status/Shooter", shooter.isConnected());
    Logger.recordOutput("Status/BallPath", ballPath.isConnected());
    Logger.recordOutput("Status/IntakeDeploy", intakeDeploy.isConnected());
    Logger.recordOutput("Status/IntakeRoller", intakeRoller.isConnected());
    Logger.recordOutput("Status/Vision", vision.allCamerasConnected());
    Logger.recordOutput("Status/Leds", leds.isConnected());
    updateLeds();
    Logger.recordOutput("Status/TurretHomed", turret.isHomed());
    Logger.recordOutput("Status/Auto", autoChooser != null && autoChooser.get() != null);

    Logger.recordOutput("Pass/SelectedBoth", passSide == FieldConstants.PassSide.BOTH);
    Logger.recordOutput("Pass/SelectedLeft", passSide == FieldConstants.PassSide.LEFT);
    Logger.recordOutput("Pass/SelectedRight", passSide == FieldConstants.PassSide.RIGHT);
  }

  /**
   * Green while fuel is going through, a fast yellow blink once the turret is on target, red while
   * it is not. Passing uses the same three, since the question it answers is the same.
   */
  private void updateLeds() {
    if (!DriverStation.isEnabled()) {
      leds.solid(LedConstants.kStartupColor, "Startup");
    } else if (feeding) {
      leds.solid(LedConstants.kFeedingColor, "Feeding");
    } else if (turret.atGoal(TurretConstants.kStartShotToleranceDegrees)) {
      leds.strobe(LedConstants.kReadyColor, LedConstants.kReadyStrobeHz, "Ready");
    } else {
      leds.solid(LedConstants.kNotAlignedColor, "NotAligned");
    }
  }

  /** Rumbles harder the further the battery sags, so the driver can back off before a brownout. */
  private void updateBrownoutRumble() {
    double volts = rumbleVoltageFilter.calculate(RobotController.getBatteryVoltage());
    double intensity =
        DriverStation.isTeleopEnabled()
            ? MathUtil.clamp(
                (kRumbleStartVolts - volts) / (kRumbleStartVolts - kRumbleFullVolts), 0.0, 1.0)
            : 0.0;
    driverController.getHID().setRumble(RumbleType.kBothRumble, intensity);
    Logger.recordOutput("Driver/RumbleBatteryVolts", volts);
    Logger.recordOutput("Driver/RumbleIntensity", intensity);
  }

  private static BooleanEntry[] passEntries() {
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Pass");
    var sides = FieldConstants.PassSide.values();
    var entries = new BooleanEntry[sides.length];
    for (int i = 0; i < sides.length; i++) {
      entries[i] = table.getBooleanTopic(sides[i].label).getEntry(false);
      // Publish immediately so the topic exists in the dashboard tree before the first loop.
      entries[i].set(sides[i] == PassSide.BOTH);
    }
    return entries;
  }

  /** Radio-button behaviour: a button the dashboard switches on wins, and one is always on. */
  private void updatePassSelection() {
    var sides = PassSide.values();
    for (int i = 0; i < sides.length; i++) {
      // Any button reading true that is not the armed side is a click that just arrived.
      if (passEntries[i].get() && sides[i] != passSide) {
        passSide = sides[i];
      }
    }

    // Write only when something actually disagrees. Publishing all three every loop overwrites
    // the dashboard's click in the same 20 ms it was made, so the button never appears to take.
    boolean stale = passSide != lastPublishedSide;
    for (int i = 0; i < sides.length && !stale; i++) {
      stale = passEntries[i].get() != (sides[i] == passSide);
    }
    if (stale) {
      for (int i = 0; i < sides.length; i++) {
        passEntries[i].set(sides[i] == passSide);
      }
      lastPublishedSide = passSide;
    }
  }

  /** Holds the intake out, whether or not the travel has been calibrated. */
  private Command holdDeployed() {
    return IntakeDeploy.isTravelMeasured() ? intakeDeploy.deploy() : intakeDeploy.jog(() -> 1.0);
  }

  /** Deploy and collect. Claims no drive subsystem, so the trajectory keeps running underneath. */
  private Command autoIntake() {
    return Commands.parallel(holdDeployed(), intakeRoller.intake()).withName("AutoIntake");
  }

  /**
   * Rollers off with the intake left out. Holding the deploy is the point -- releasing it would
   * hand the subsystem to its default command, which retracts.
   */
  private Command autoIntakeStop() {
    return Commands.parallel(holdDeployed(), intakeRoller.idle()).withName("AutoIntakeStop");
  }

  /**
   * Shoot on the move, with the same intake behaviour as a teleop shot: the deploy walks home
   * over kShootStow seconds instead of snapping back, and the roller runs shoot assist rather
   * than intake speed.
   */
  private Command autoShoot() {
    return Commands.parallel(shoot(), intakeDeploy.shootStow(), intakeRoller.shootAssist())
        .withName("AutoShoot");
  }

  /**
   * Pre-spin, and put the turret on the hub itself rather than on a lookahead, so the shot only has
   * to close the last of the flywheel ramp and none of the aim. The turret tracks all through auto
   * on its default command already; this only changes what it is tracking.
   */
  private Command autoPrepShoot() {
    return shooter
        .setRpm(ShooterConstants.kSpinUpRpm)
        .beforeStarting(() -> prepping = true)
        .finallyDo(() -> prepping = false)
        .withName("AutoPrepShoot");
  }

  /**
   * The B button's recall, available to a path. Ends on its own once the turret is home, so tracking
   * picks straight back up and nothing is left running when the auto finishes.
   */
  private Command autoTurretZero() {
    return turret.goToZero().withName("AutoTurretZero");
  }

  /** Ends the shot by taking the hood, and does not finish until the hood is actually stowed. */
  private Command autoStopShoot() {
    return hood.setAngle(HoodConstants.kStowDegrees)
        .until(() -> hood.atGoal(HoodConstants.kStowDegrees))
        .withName("AutoStopShoot");
  }

  /**
   * Shared by both path libraries: PathPlanner event markers look these up by name, and Choreo
   * routines bind the same names to trajectory event triggers.
   */
  private void registerAutoCommands(AutoFactory factory) {
    NamedCommands.registerCommand("PrepShoot", autoPrepShoot());
    NamedCommands.registerCommand("Intake", autoIntake());
    NamedCommands.registerCommand("IntakeStop", autoIntakeStop());
    NamedCommands.registerCommand("Shoot", autoShoot());
    NamedCommands.registerCommand("StopShoot", autoStopShoot());
    NamedCommands.registerCommand("TurretZero", autoTurretZero());
    factory.bind("PrepShoot", autoPrepShoot());
    factory.bind("Intake", autoIntake());
    factory.bind("IntakeStop", autoIntakeStop());
    factory.bind("Shoot", autoShoot());
    factory.bind("StopShoot", autoStopShoot());
    factory.bind("TurretZero", autoTurretZero());
  }

  /**
   * Every .traj in deploy/choreo becomes a drive-only option. Choreo has no equivalent of
   * PathPlanner's auto discovery, and hand-listing names meant a new path silently never appeared.
   */
  private void addChoreoRoutines(LoggedDashboardChooser<Command> chooser) {
    String[] files =
        new File(Filesystem.getDeployDirectory(), "choreo")
            .list((dir, name) -> name.endsWith(".traj"));
    if (files == null) {
      return;
    }
    Arrays.sort(files);
    for (String file : files) {
      String name = file.substring(0, file.length() - ".traj".length());
      chooser.addOption(name, driveTrajectory(name, false));
      // The field is symmetric across its width, so a Right path mirrors into a Left one.
      String mirrored = mirroredName(name);
      if (mirrored != null) {
        chooser.addOption(mirrored, driveTrajectory(name, true));
      }
    }
    Logger.recordOutput("Auto/ChoreoTrajectories", files.length);
  }

  /** Resets odometry to the trajectory's start, then drives it. Choreo never resets on its own. */
  private Command driveTrajectory(String name, boolean mirror) {
    var routine = autoFactory.newRoutine(mirror ? mirroredName(name) : name);
    AutoTrajectory trajectory = routine.trajectory(name);
    if (mirror) {
      trajectory = trajectory.mirrorY();
    }
    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
    return routine.cmd();
  }

  /** Null when the name says nothing about a side, so nothing is mirrored blindly. */
  private static String mirroredName(String name) {
    if (name.contains("Right")) {
      return name.replace("Right", "Left");
    }
    if (name.contains("Left")) {
      return name.replace("Left", "Right");
    }
    return null;
  }

  public Command getAutonomousCommand() {
    if (autoChooser == null) {
      return Commands.none();
    }
    Command selected = autoChooser.get();
    return selected == null ? Commands.none() : selected;
  }
  /**
   * What each mechanism costs the battery, so the biggest draw can be found rather than guessed at.
   *
   * <p>Drive is listed first because it is nearly always the answer. Anything not on a motor
   * controller -- the RIO, the radio, the cameras -- is not here, so the total is what the
   * mechanisms cost rather than what the robot draws.
   */
  public void updatePowerMonitor() {
    if (drive != null) {
      powerMonitor.record("Drive", drive.supplyCurrentAmps());
    }
    powerMonitor.record("Shooter", shooter.supplyCurrentAmps());
    powerMonitor.record("BallPath", ballPath.supplyCurrentAmps());
    powerMonitor.record("IntakeRoller", intakeRoller.supplyCurrentAmps());
    powerMonitor.record("IntakeDeploy", intakeDeploy.supplyCurrentAmps());
    powerMonitor.record("Turret", turret.supplyCurrentAmps());
    powerMonitor.record("Hood", hood.supplyCurrentAmps());
    powerMonitor.publish();
  }

}
