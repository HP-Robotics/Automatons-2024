// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;


import java.util.function.DoubleToIntFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTableEntry botpose_blue;
  NetworkTable m_gamePieceTable;
  NetworkTable m_limelight_twoplus;
  public boolean m_sawAprilTag;
  public boolean m_aprilTagSeen;
  public int m_targetAprilTagID;
  final DoubleArraySubscriber limeSub;
  private double[] defaultValues = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  /** Creates a new ExampleSubsystem. */
  public Pose2d m_visionPose2d = new Pose2d();
  public Pose2d m_robotPose;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelightMagicTable = inst.getTable("limelight-subsystem");
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> publisher;

  PoseEstimatorSubsystem m_poseEstimator;

  public LimelightSubsystem(PoseEstimatorSubsystem PoseEstimatorSubsystem) {
    m_limelight_twoplus = NetworkTableInstance.getDefault().getTable("limelight-fight");
    botpose_blue = m_limelight_twoplus.getEntry("botpose_wpiblue"); // TODO: Look into Megatag
    m_poseEstimator = PoseEstimatorSubsystem;
    publisher = poseEstimatorTable.getStructTopic("AprilTagPose", Pose2d.struct).publish();
    limeSub = m_limelight_twoplus.getDoubleArrayTopic("botpose_wpiblue").subscribe(defaultValues);

    m_gamePieceTable = NetworkTableInstance.getDefault().getTable("limelight-bite");
    m_aprilTagSeen = false;
  }

  public double getDistanceTo(Pose2d robot, Pose2d fieldpose) {
    double turnToSpeakerA = fieldpose.getX() - robot.getX();
    double turnToSpeakerB = fieldpose.getY() - robot.getY();
    double distanceToSpeaker = Math.sqrt(Math.pow(turnToSpeakerA, 2) + Math.pow(turnToSpeakerB, 2));

    return distanceToSpeaker;
  }

  public double getAngleTo(Pose2d robot, Pose2d fieldpose) {
    double turnToSpeakerA = fieldpose.getX() - robot.getX();
    double turnToSpeakerB = fieldpose.getY() - robot.getY();
    double angleToSpeaker = Math.atan2(turnToSpeakerB, turnToSpeakerA);

    return Math.toDegrees(angleToSpeaker);
  }

  public Optional<Double> getNoteTX() {
    double noteTV = m_gamePieceTable.getEntry("tv").getValue().getDouble(); // TODO: Is this safe to call if we don't have a limelight?
    double noteTX = m_gamePieceTable.getEntry("tx").getValue().getDouble();

    if (noteTV == 1) {
      limelightMagicTable.putValue("noteTX", NetworkTableValue.makeDouble(noteTX));
      return Optional.of(noteTX);
    } else {
      return Optional.empty();
    }
  }

  // This method will be called once per scheduler run
  public void periodic() {
    // read values periodically
    double[] botpose = null;
    double Stamp = 0;
    double latency = 0;
    double timeStamp = 0;
    m_sawAprilTag = m_limelight_twoplus.getEntry("tv").getDouble(0) == 1.0;
    limelightMagicTable.putValue("Saw April Tag", NetworkTableValue.makeBoolean(m_sawAprilTag));
    if (m_sawAprilTag) {
      m_targetAprilTagID = (int) m_limelight_twoplus.getEntry("tid").getInteger(0);
      for (TimestampedDoubleArray tsValue : limeSub.readQueue()) {
        botpose = tsValue.value;
        timeStamp = Timer.getFPGATimestamp();
        double tx = botpose[0];
        double ty = botpose[1];
        // double tz = botpose[2];
        // double rx = botpose[3];
        // double ry = botpose[4]; // TODO: Store tz, rx, and ry somewhere (Store 3D)
        double rz = botpose[5];
        latency = botpose[6];
        timeStamp -= latency / 1000;
        m_robotPose = new Pose2d(tx, ty, new Rotation2d(Math.toRadians(rz)));
        m_visionPose2d = m_robotPose;
        if (m_poseEstimator != null && 0 <= m_targetAprilTagID && m_targetAprilTagID <= 16) {
          if (botpose[0] != 0 || botpose[1] != 0 || botpose[5] != 0) {
            publisher.set(m_robotPose);
            limelightMagicTable.putValue("poseEstimator Timestamp", NetworkTableValue.makeDouble(timeStamp));
            limelightMagicTable.putValue("current Timestamp", NetworkTableValue.makeDouble(Timer.getFPGATimestamp()));
            m_poseEstimator.updateVision(m_robotPose, timeStamp,
                getDistanceTo(m_robotPose, LimelightConstants.aprilTagList[m_targetAprilTagID]));
            // System.out.println("saw apriltag: " + timeStamp + " latency: " + latency);
          }
        }
      }
      limelightMagicTable.putValue(
          "distanceToSpeaker",
          NetworkTableValue.makeDouble(getDistanceTo(m_robotPose, LimelightConstants.aprilTagList[7])));
      limelightMagicTable.putValue(
          "angleToSpeaker",
          NetworkTableValue.makeDouble(getAngleTo(m_robotPose, LimelightConstants.aprilTagList[7])));

      if (!m_aprilTagSeen) {
        m_aprilTagSeen = true;
        //TODO: maybe run poseEstimatorSubsystem's resetPosition
      }

      // double noteTV = m_gamePieceTable.getEntry("tv").getValue().getDouble();
      // double noteTX = m_gamePieceTable.getEntry("tx").getValue().getDouble();
      // double noteTY = m_gamePieceTable.getEntry("ty").getValue().getDouble();
    }
  }
}
