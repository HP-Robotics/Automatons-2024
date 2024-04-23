// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

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
    botpose_blue = m_limelight_twoplus.getEntry("botpose_orb_wpiblue"); // TODO: Look into Megatag
    m_poseEstimator = PoseEstimatorSubsystem;
    publisher = poseEstimatorTable.getStructTopic("AprilTagPose", Pose2d.struct).publish();
    limeSub = m_limelight_twoplus.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(defaultValues);

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
    double noteTV = m_gamePieceTable.getEntry("tv").getDouble(0.0);
    double noteTX = m_gamePieceTable.getEntry("tx").getDouble(0.0);

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
    double[] entries = new double[6];
        entries[0] = m_poseEstimator.getPose().getRotation().getDegrees();
        entries[1] = 0;
        entries[2] = 0;
        entries[3] = 0;
        entries[4] = 0;
        entries[5] = 0;
    m_limelight_twoplus.getEntry("robot_orientation_set").setDoubleArray(entries);
    double[] botpose = null;
    double latency = 0;
    double timeStamp = 0;
    double[] botposeEmpty = {0, 0, 0, 0, 0, 0, 0, 0};
    m_sawAprilTag = m_limelight_twoplus.getEntry("botpose_orb_wpiblue").getDoubleArray(botposeEmpty)[7] != 0;
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
        if (m_poseEstimator != null && 0 <= m_targetAprilTagID && m_targetAprilTagID <= 16 && m_targetAprilTagID !=8 && m_targetAprilTagID !=3) {
          limelightMagicTable.putValue(
              "distanceToSpeaker",
              NetworkTableValue
                  .makeDouble(getDistanceTo(m_robotPose, LimelightConstants.aprilTagList[m_targetAprilTagID])));
          limelightMagicTable.putValue(
              "angleToSpeaker",
              NetworkTableValue
                  .makeDouble(getAngleTo(m_robotPose, LimelightConstants.aprilTagList[m_targetAprilTagID])));
          if (botpose[0] != 0 || botpose[1] != 0 || botpose[5] != 0) {
            publisher.set(m_robotPose);
            limelightMagicTable.putValue("poseEstimator Timestamp", NetworkTableValue.makeDouble(timeStamp));
            limelightMagicTable.putValue("current Timestamp", NetworkTableValue.makeDouble(Timer.getFPGATimestamp()));
            double skew = LimelightConstants.aprilTagList[m_targetAprilTagID].getRotation()
                .minus(Rotation2d
                    .fromDegrees(getAngleTo(m_robotPose, LimelightConstants.aprilTagList[m_targetAprilTagID])-180))
                .getRadians();
            skew = Math.abs(skew);
            m_poseEstimator.updateVision(m_robotPose, timeStamp,
                getDistanceTo(m_robotPose, LimelightConstants.aprilTagList[m_targetAprilTagID]), skew);
            // System.out.println("saw apriltag: " + timeStamp + " latency: " + latency);
          }
        }
      }

      if (!m_aprilTagSeen) {
        m_aprilTagSeen = true;
        // TODO: maybe run poseEstimatorSubsystem's resetPosition
      }
      // double noteTV = m_gamePieceTable.getEntry("tv").getValue().getDouble();
      // double noteTX = m_gamePieceTable.getEntry("tx").getValue().getDouble();
      // double noteTY = m_gamePieceTable.getEntry("ty").getValue().getDouble();
    }
  }
}
