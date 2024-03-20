// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class DrivePointedToSpeakerCommand extends Command {
  private final DriveSubsystem m_drivesubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;
  private final CommandJoystick m_joystick;
  private Pose2d m_targetAprilTag;
  private Rotation2d m_offset;
  private boolean m_aprilTagSeen = false;

  /** Creates a new IntakeCommand. */
  public DrivePointedToSpeakerCommand(DriveSubsystem drivesubsystem, LimelightSubsystem limelightsubsystem,
      PoseEstimatorSubsystem poseestimatorsubsystem,
      CommandJoystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivesubsystem = drivesubsystem;
    m_limelightSubsystem = limelightsubsystem;
    m_poseEstimatorSubsystem = poseestimatorsubsystem;
    m_joystick = joystick;
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
    if (m_limelightSubsystem.m_aprilTagSeen) {
      m_drivesubsystem.drivePointedTowardsAngle(m_joystick,
          new Rotation2d(Math
              .toRadians(m_limelightSubsystem.getAngleTo(m_poseEstimatorSubsystem.getPose(), m_targetAprilTag)) + Math.PI));
      if (m_limelightSubsystem.getDistanceTo(m_poseEstimatorSubsystem.getPose(), m_targetAprilTag) < LimelightConstants.allowableSpeakerDistanceError) {
        m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0.2);
      } else {
        m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      m_drivesubsystem.driveWithJoystick(m_joystick);
      m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
