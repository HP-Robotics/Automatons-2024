// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;
import frc.robot.TriangleInterpolator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class DrivePointedToSpeakerCommand extends Command {
  private final DriveSubsystem m_drivesubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private final TriangleInterpolator m_triangleInterpolator;
  private Optional<CommandJoystick> m_joystick;
  private Pose2d m_targetAprilTag;

  /** Creates a new IntakeCommand. */
  public DrivePointedToSpeakerCommand(DriveSubsystem drivesubsystem, LimelightSubsystem limelightsubsystem,
      PoseEstimatorSubsystem poseestimatorsubsystem,
      CommandJoystick joystick, TriangleInterpolator magicTriangles) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivesubsystem = drivesubsystem;
    m_limelightSubsystem = limelightsubsystem;
    m_poseEstimatorSubsystem = poseestimatorsubsystem;
    m_joystick = Optional.empty();
    if (joystick != null) {
      m_joystick = Optional.of(joystick);
    }
    m_triangleInterpolator = magicTriangles;
    addRequirements(drivesubsystem);
  }

  public DrivePointedToSpeakerCommand(DriveSubsystem drivesubsystem, LimelightSubsystem limelightsubsystem,
      PoseEstimatorSubsystem poseestimatorsubsystem, TriangleInterpolator magicTriangles) {
    // Use addRequirements() here to declare subsystem dependencies.

    this(drivesubsystem, limelightsubsystem, poseestimatorsubsystem, null, magicTriangles);
    addRequirements(drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    double rumble = 0;
    Pose2d currentPose = m_poseEstimatorSubsystem.getAlliancePose();
    if (/* m_limelightSubsystem.m_aprilTagSeen && */ currentPose != null) { // TODO: Make this less of a mess
      Optional<double[]> triangleData = m_triangleInterpolator.getTriangulatedOutput(currentPose);

      Rotation2d heading = new Rotation2d(Math
          .toRadians(m_limelightSubsystem.getAngleTo(m_poseEstimatorSubsystem.getPose(), m_targetAprilTag))
          + Math.PI);
      if (triangleData.isPresent()) { // If we have magic data
        rumble = 0.4;
        heading = new Rotation2d(triangleData.get()[3]);
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
          heading = GeometryUtil.flipFieldRotation(heading);
        }
      }
      if (m_joystick.isPresent()) {
        m_drivesubsystem.drivePointedTowardsAngle(m_joystick.get(), heading);
      } else {
        m_drivesubsystem.driveForwardWithAngle(0, heading);
      }
    } else {
      if (m_joystick.isPresent()) {
        m_drivesubsystem.driveWithJoystick(m_joystick.get());
      }
    }
    if (m_joystick.isPresent()) {
      m_joystick.get().getHID().setRumble(RumbleType.kBothRumble, rumble);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_joystick.isPresent()) {
      m_joystick.get().getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_joystick.isEmpty() && m_drivesubsystem.pointedTowardsAngle()) {
      return true;
    }
    return false;
  }
}
