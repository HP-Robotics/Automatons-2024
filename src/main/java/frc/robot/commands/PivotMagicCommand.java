// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TriangleInterpolator;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PivotMagicCommand extends Command {
  private final PivotSubsystem m_subsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private Pose2d m_targetAprilTag;
  private TriangleInterpolator m_triangleInterpolator;
  private TriangleInterpolator m_feederInterpolator;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable pivotTable = inst.getTable("pivot-table");

  /** Creates a new pivotMagicCommand. */
  public PivotMagicCommand(PivotSubsystem subsystem, LimelightSubsystem limelightSubsystem,
      TriangleInterpolator magicTriangles, TriangleInterpolator feederInterpolator, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_triangleInterpolator = magicTriangles;
    m_feederInterpolator = feederInterpolator;
    m_poseEstimatorSubsystem = poseEstimatorSubsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Started Magic owo");
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        m_targetAprilTag = LimelightConstants.aprilTagList[7];
      } else {
        m_targetAprilTag = LimelightConstants.aprilTagList[4];
      }
    } else {
      m_targetAprilTag = LimelightConstants.aprilTagList[7];
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_poseEstimatorSubsystem.getAlliancePose();
    if (currentPose != null) {
      // System.out.println("Getting Data");
      Optional<double[]> interpolatorData = m_triangleInterpolator.getTriangulatedOutput(currentPose);
      if (interpolatorData.isEmpty()) {
        interpolatorData = m_feederInterpolator.getTriangulatedOutput(currentPose);
      }

      if (interpolatorData.isPresent()) {
        // System.out.println("Recived Data");
        m_subsystem.setPosition(interpolatorData.get()[2]);
      }
      // m_subsystem.setPosition(m_subsystem.getMagicAngle(
      // m_limelightSubsystem.getDistanceTo(m_limelightSubsystem.m_visionPose2d,
      // m_targetAprilTag))); */
      pivotTable.putValue(
          "magicEncoderValue", NetworkTableValue.makeDouble(m_subsystem.getMagicAngle(
              m_limelightSubsystem.getDistanceTo(currentPose, m_targetAprilTag))));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
