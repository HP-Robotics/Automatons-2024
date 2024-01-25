// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class LimelightSubsystem extends SubsystemBase {
  NetworkTableEntry botpose_blue;
  /** Creates a new ExampleSubsystem. */
  public final Field2d m_field = new Field2d();
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
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
    

    //read values periodically
    double defaultValues[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double[] botpose = botpose_blue.getDoubleArray(defaultValues);
    double tx = botpose[0];
    double ty = botpose[1];
    double tz = botpose[2];
    double rx = botpose[3];
    double ry = botpose[4];
    double rz = botpose[5];
    
    Pose2d m_robotPose = new Pose2d(tx,ty,new Rotation2d(Math.toRadians(rz)));
    m_field.setRobotPose(m_robotPose);
    // specify the widget here
    // m_field.setRobotPose(table.getdoub.getPoseMeters());
    // System.out.println(tx);
    // System.out.println(ty);


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}