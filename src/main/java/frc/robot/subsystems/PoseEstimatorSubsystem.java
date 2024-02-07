// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class PoseEstimatorSubsystem extends SubsystemBase {
  SwerveDriveKinematics m_kinematics;
  Rotation2d m_gyroAngle;
  SwerveModulePosition[] m_swervePositions;
  Pose2d m_initialPose;
  SwerveDrivePoseEstimator poseEstimator;
  
  public PoseEstimatorSubsystem() {
  SwerveDriveKinematics m_kinematics;
  Rotation2d m_gyroAngle;
  SwerveModulePosition[] m_swervePositions;
  Pose2d m_initialPose;
   //make fake intializaition run in Drive Subsystem
  }



  public void createPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d angle,SwerveModulePosition[] swervePositions,Pose2d initialPose) {
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, angle, swervePositions, initialPose);
  }

  public void updatePoseEstimator(Rotation2d angle, SwerveModulePosition[] positions){
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), angle, positions);
  }

  public void updateVision(Pose2d vPose,double vTime){
    poseEstimator.addVisionMeasurement(vPose, vTime);
  }
  
}
