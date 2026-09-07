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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ballpath.BallPath;
import frc.robot.subsystems.ballpath.BallPathIO;
import frc.robot.subsystems.ballpath.BallPathIOTalonFX;
import frc.robot.shooting.ShotMap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeRoller;
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

public class RobotContainer {
  private final Vision vision;
  private final Turret turret;
  private final BallPath ballPath;
  private final Hood hood;
  private final Shooter shooter;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;

  private final CommandSwerveDrivetrain drive;

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);

  private static final double kStickDeadband = 0.1;

  private static final double kDriveSpeedScale = 0.4;

  private static final double kDriveTurnScale = 0.8;

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
                return driveRequest
                    .withVelocityX(translation.getX() * maxSpeed * kDriveSpeedScale)
                    .withVelocityY(translation.getY() * maxSpeed * kDriveSpeedScale)
                    .withRotationalRate(
                        shapedAxis(-driverController.getRightX())
                            * maxAngularRate
                            * kDriveTurnScale);
              }));
    }

    driverController
        .rightTrigger(0.5)
        .whileTrue(
            Commands.parallel(
                shooter.setRpm(() -> currentShot().shooterRpm()),
                hood.setAngle(() -> currentShot().hoodDegrees()),
                Commands.waitUntil(
                        () ->
                            shooter.readyToFeed()
                                && hood.atGoal(currentShot().hoodDegrees()))
                    .andThen(
                        Commands.parallel(
                            ballPath.runAll(),
                            intakeDeploy.shootPulse(),
                            intakeRoller.shootAssist()))));

    // Start held is a modifier: Start+A/B characterise the intake pivot.
    Trigger sysIdMode = driverController.start();
    sysIdMode
        .and(driverController.a())
        .whileTrue(intakeDeploy.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    sysIdMode
        .and(driverController.b())
        .whileTrue(intakeDeploy.sysIdDynamic(SysIdRoutine.Direction.kForward));

    Trigger intakeButton = driverController.leftTrigger(0.5);
    intakeButton.whileTrue(intakeRoller.intake());
    intakeButton.whileTrue(ballPath.loadIndexer());
    if (IntakeDeploy.isTravelMeasured()) {
      intakeButton.onTrue(intakeDeploy.deploy());
      driverController.y().onTrue(intakeDeploy.retract());
    } else {
      intakeButton.whileTrue(intakeDeploy.jog(() -> 1.0));
    }

    driverController.back().onTrue(intakeDeploy.retract());

    driverController.povUp().whileTrue(hood.jog(() -> 1.0));
    driverController.povDown().whileTrue(hood.jog(() -> -1.0));

    driverController.x().whileTrue(ballPath.runSingulator());
    driverController.y().whileTrue(ballPath.runIndexer());

    driverController.povRight().whileTrue(turret.jog(() -> 1.0));
    driverController.povLeft().whileTrue(turret.jog(() -> -1.0));
    driverController
        .a()
        .whileTrue(shooter.setRpm(ShooterConstants.kTestRpm));

    driverController
        .leftBumper()
        .whileTrue(
            turret.setAngle(
                () -> fieldRelativeTurretBearing(bearingToGoalDegrees()),
                this::turretBearingRateDegPerSec));

    driverController.rightBumper().onTrue(turret.declareZero());

    if (drive != null) {
      driverController.b().onTrue(drive.runOnce(drive::seedFieldCentric).ignoringDisable(true));
    }
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
  }

  private double turretBearingRateDegPerSec() {
    if (drive == null) {
      return 0.0;
    }
    var state = drive.getState();
    Translation2d toGoal = FieldConstants.ourGoal().minus(turretFieldPosition());
    double distanceSquared = toGoal.getNorm() * toGoal.getNorm();
    if (distanceSquared < 1e-6) {
      return 0.0;
    }

    Translation2d fieldVelocity =
        new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)
            .rotateBy(state.Pose.getRotation());

    double bearingRate =
        (toGoal.getY() * fieldVelocity.getX() - toGoal.getX() * fieldVelocity.getY())
            / distanceSquared;

    double turretRate = bearingRate - state.Speeds.omegaRadiansPerSecond;
    Logger.recordOutput("Turret/BearingRateDegPerSec", Math.toDegrees(bearingRate));
    return Math.toDegrees(TurretConstants.kPositiveMatchesGyro ? turretRate : -turretRate);
  }

  private Translation2d turretFieldPosition() {
    var pose = drive.getState().Pose;
    Translation2d position =
        pose.getTranslation()
            .plus(TurretConstants.kRobotToTurret.rotateBy(pose.getRotation()));
    Logger.recordOutput("Turret/PivotFieldPosition", position);
    return position;
  }

  private ShotMap.Shot currentShot() {
    if (drive == null) {
      return ShotMap.lookup(0.0);
    }
    double distance = FieldConstants.ourGoal().minus(turretFieldPosition()).getNorm();
    ShotMap.Shot shot = ShotMap.lookup(distance);
    Logger.recordOutput("Shot/DistanceMeters", distance);
    Logger.recordOutput("Shot/MapHoodDegrees", shot.hoodDegrees());
    Logger.recordOutput("Shot/MapShooterRpm", shot.shooterRpm());
    return shot;
  }

  private double bearingToGoalDegrees() {
    if (drive == null) {
      return 0.0;
    }
    Translation2d toGoal = FieldConstants.ourGoal().minus(turretFieldPosition());
    Logger.recordOutput("Turret/GoalTranslation", FieldConstants.ourGoal());
    Logger.recordOutput("Turret/DistanceToGoalMeters", toGoal.getNorm());
    return toGoal.getAngle().getDegrees();
  }
  private double fieldRelativeTurretBearing(double fieldDegrees) {
    double chassisHeadingDegrees =
        drive == null ? 0.0 : drive.getState().Pose.getRotation().getDegrees();
    Logger.recordOutput("Turret/ChassisHeadingDegrees", chassisHeadingDegrees);

    double bearing = fieldDegrees - chassisHeadingDegrees;
    return (TurretConstants.kPositiveMatchesGyro ? bearing : -bearing)
        + TurretConstants.kForwardOffsetDegrees;
  }

  private double jogInput() {
    return driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis();
  }

  public Command getAutonomousCommand() {
    // TODO(10015): build from PathPlanner's AutoBuilder once the drivetrain is generated.
    return Commands.none();
  }
}
