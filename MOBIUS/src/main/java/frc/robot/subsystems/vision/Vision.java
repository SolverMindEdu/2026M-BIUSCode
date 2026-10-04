// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private boolean allCamerasConnected = false;
  private final VisionIOInputsAutoLogged[] inputs;

  private double lastAcceptedPoseTimestamp = Double.NEGATIVE_INFINITY;
  private double lastMultiTagTimestamp = Double.NEGATIVE_INFINITY;
  private boolean defenseMode = false;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  public boolean hasRecentPoseVision() {
    return Logger.getTimestamp() / 1.0e6 - lastAcceptedPoseTimestamp < poseVisionTimeoutSecs;
  }

  /** True while the wheels are X-locked, where odometry misses shoves and vision translation is trusted more. */
  public void setDefenseMode(boolean defenseMode) {
    this.defenseMode = defenseMode;
  }

  /** True while some camera has recently had two or more tags in one accepted pose. */
  public boolean hasRecentMultiTagVision() {
    return Logger.getTimestamp() / 1.0e6 - lastMultiTagTimestamp < multiTagTimeoutSecs;
  }

  @Override
  public void periodic() {
    int connectedCount = 0;
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
      if (inputs[i].connected) {
        connectedCount++;
      }
    }
    // One flag for the dashboard, plus the count so a single dropout is identifiable.
    allCamerasConnected = connectedCount == io.length;
    Logger.recordOutput("Vision/AllCamerasConnected", allCamerasConnected);
    Logger.recordOutput("Vision/ConnectedCameraCount", connectedCount);

    NetworkTableInstance.getDefault().flush();

    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      boolean seesAimTag = false;
      for (int tagId : inputs[cameraIndex].tagIds) {
        seesAimTag |= FieldConstants.isAimTag(tagId);
      }
      Logger.recordOutput("Vision/Camera" + cameraIndex + "/SeesAimTag", seesAimTag);

      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean megatag1 = observation.type() == PoseObservationType.MEGATAG_1;
        boolean enabled = DriverStation.isEnabled();
        // While enabled, only clean, close, multi-tag MegaTag1 reads may touch the heading.
        boolean untrustedHeading =
            megatag1
                && enabled
                && (observation.tagCount() < 2
                    || observation.ambiguity() > megatag1MaxAmbiguity
                    || observation.averageTagDistance() > megatag1MaxDistanceMeters);
        boolean rejectPose =
            (kRequireAimTags && !seesAimTag)
                || untrustedHeading
                || observation.tagCount() == 0
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity)
                || Math.abs(observation.pose().getZ())
                    > maxZError

                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        if (rejectPose) {
          continue;
        }

        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double cameraFactor =
            cameraIndex < cameraStdDevFactors.length ? cameraStdDevFactors[cameraIndex] : 1.0;
        // MegaTag2 gives translation only and MegaTag1 gives heading only; infinite uncertainty ignores the other part.
        double linearStdDev =
            megatag1
                ? Double.POSITIVE_INFINITY
                : linearStdDevBaseline
                    * linearStdDevMegatag2Factor
                    * stdDevFactor
                    * cameraFactor
                    * (defenseMode ? defenseLinearStdDevFactor : 1.0);
        double angularStdDev =
            megatag1
                ? angularStdDevBaseline
                    * stdDevFactor
                    * cameraFactor
                    * (enabled ? megatag1EnabledAngularFactor : 1.0)
                : Double.POSITIVE_INFINITY;

        lastAcceptedPoseTimestamp = Logger.getTimestamp() / 1.0e6;
        if (observation.tagCount() >= 2) {
          lastMultiTagTimestamp = lastAcceptedPoseTimestamp;
        }
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      if (kLogPerCameraPoses) {
        String key = "Vision/Camera" + Integer.toString(cameraIndex);
        Logger.recordOutput(key + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(key + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(key + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(key + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
      }
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    Logger.recordOutput("Vision/Summary/HasRecentPoseVision", hasRecentPoseVision());
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
  public boolean allCamerasConnected() {
    return allCamerasConnected;
  }
}
