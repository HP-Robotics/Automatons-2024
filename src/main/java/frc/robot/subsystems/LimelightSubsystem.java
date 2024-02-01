// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTableEntry botpose_blue;
  /** Creates a new ExampleSubsystem. */
  public final Field2d m_field = new Field2d();

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelightMagicTable = inst.getTable("limelight Magic numbers");

  public LimelightSubsystem() {
    Shuffleboard.getTab("shuffleboard")
        .add("Pose2d", m_field)
        .withWidget(BuiltInWidgets.kField);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-three");
    botpose_blue = table.getEntry("botpose_wpiblue");
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read values periodically
    double defaultValues[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double[] botpose = botpose_blue.getDoubleArray(defaultValues);
    double tx = botpose[0];
    double ty = botpose[1];
    double tz = botpose[2];
    double rx = botpose[3];
    double ry = botpose[4];
    double rz = botpose[5];

    Pose2d m_robotPose = new Pose2d(tx, ty, new Rotation2d(Math.toRadians(rz)));
    m_field.setRobotPose(m_robotPose);
    // specify the widget here
    // m_field.setRobotPose(table.getdoub.getPoseMeters());
    // System.out.println(tx);
    // System.out.println(ty);

    getDistanceTo(m_robotPose, LimelightConstants.aprilTag7);
    limelightMagicTable.putValue(
      "distanceToSpeaker", NetworkTableValue.makeDouble(getDistanceTo(m_robotPose, LimelightConstants.aprilTag7)));
    limelightMagicTable.putValue(
      "angleToSpeaker", NetworkTableValue.makeDouble(getAngleTo(m_robotPose, LimelightConstants.aprilTag7)));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
