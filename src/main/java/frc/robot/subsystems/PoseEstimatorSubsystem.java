// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PoseEstimatorConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");
  StructPublisher<Pose2d> posePublisher;

  SwerveDrivePoseEstimator poseEstimator;

  public PoseEstimatorSubsystem() {
    posePublisher = poseEstimatorTable.getStructTopic("Fused Pose", Pose2d.struct).publish();

    // make fake intializaition run in Drive Subsystem
  }

  public void createPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d angle,
      SwerveModulePosition[] swervePositions, Pose2d initialPose) {
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, angle, swervePositions, initialPose,
        PoseEstimatorConstants.statesStandardDev,
        VecBuilder.fill(0.1, 0.1, 0.1));
  }

  public void updatePoseEstimator(Rotation2d angle, SwerveModulePosition[] positions) {
    if (poseEstimator != null) {
      poseEstimator.updateWithTime(Timer.getFPGATimestamp(), angle, positions);
    }
  }

  public void updateVision(Pose2d vPose, double vTime, double distance) {
    if (poseEstimator != null) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
          PoseEstimatorConstants.visionXStandardDev * distance,
          PoseEstimatorConstants.visionYStandardDev * distance,
          PoseEstimatorConstants.visionHeadingStandardDev));
      poseEstimator.addVisionMeasurement(vPose, vTime);
      // System.out.println(vTime);
    }
  }

  public Pose2d getPose() {
    if (poseEstimator != null) {
      return poseEstimator.getEstimatedPosition();
    } else {
      return null;
    }
  }

  @Override
  public void periodic() {
    if (poseEstimator != null) {
      posePublisher.set(poseEstimator.getEstimatedPosition());
      VecBuilder.fill(0, 0, 0);
    }
  }
}
