// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTableEntry botpose_blue;
  double sawAprilTag;
  /** Creates a new ExampleSubsystem. */
  public final Field2d m_field = new Field2d(); // TODO: Send Pose instead of field
  public Pose2d m_visionPose2d = new Pose2d();

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelightMagicTable = inst.getTable("limelight Magic numbers");
  NetworkTable poseEstimatorTable = inst.getTable("Pose Estimator Table");

  StructPublisher<Pose2d> publisher;

  PoseEstimatorSubsystem m_poseEstimator;

  public LimelightSubsystem(PoseEstimatorSubsystem PoseEstimatorSubsystem) {
    Shuffleboard.getTab("shuffleboard")
        .add("Pose2d", m_field)
        .withWidget(BuiltInWidgets.kField);
    sawAprilTag = NetworkTableInstance.getDefault().getTable("limelight-three").getEntry("tv").getDouble(0);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-three");
    botpose_blue = table.getEntry("botpose_wpiblue"); // TODO: Look into Megatag
    m_poseEstimator = PoseEstimatorSubsystem;
    publisher = poseEstimatorTable.getStructTopic("AprilTagPose", Pose2d.struct).publish();

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

  // This method will be called once per scheduler run
  public void periodic() {
    // read values periodically
    double defaultValues[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0};
    
    NetworkTableValue blueBotpose = botpose_blue.getValue();
    double[] botpose;
    if(blueBotpose.getType()!=NetworkTableType.kUnassigned){
      botpose = blueBotpose.getDoubleArray();
    }
    else{
      botpose = defaultValues;
    }
    
    botpose_blue.getLastChange();
    
    double tx = botpose[0];
    double ty = botpose[1];
    double tz = botpose[2];
    double rx = botpose[3];
    double ry = botpose[4]; // TODO: Store tz, rx, and ry somewhere (Store 3D)
    double rz = botpose[5];
    double latency = botpose[6];

    double timeStamp = (blueBotpose.getTime()*1.0)/1000000 - latency;

    Pose2d m_robotPose = new Pose2d(tx, ty, new Rotation2d(Math.toRadians(rz)));
    m_field.setRobotPose(m_robotPose);
    m_visionPose2d = m_robotPose;
    // specify the widget here

    if(sawAprilTag == 1){
      m_poseEstimator.updateVision(m_robotPose, timeStamp);
      System.out.println("saw apriltag: " + timeStamp);
    }

    getDistanceTo(m_robotPose, LimelightConstants.aprilTag7);
    limelightMagicTable.putValue(
      "distanceToSpeaker", NetworkTableValue.makeDouble(getDistanceTo(m_robotPose, LimelightConstants.aprilTag7)));
    limelightMagicTable.putValue(
      "angleToSpeaker", NetworkTableValue.makeDouble(getAngleTo(m_robotPose, LimelightConstants.aprilTag7)));
    
    publisher.set(m_robotPose);
  }
}
