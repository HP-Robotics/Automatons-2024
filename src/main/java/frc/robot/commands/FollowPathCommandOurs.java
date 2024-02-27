// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowPathCommandOurs extends Command {
  DriveSubsystem m_drive;
  String m_pathName;
  PathPlannerPath m_path;

  public FollowPathCommand m_pathPlannerCommand;
  public PPHolonomicDriveController m_HolonomicDriveController;

  public FollowPathCommandOurs(DriveSubsystem Drive, String PathName) {
    m_drive = Drive;
    m_pathName = PathName;

    m_path = PathPlannerPath.fromPathFile(PathName);
    m_pathPlannerCommand = PathCommand();
    addRequirements(Drive);
  }

  /** Creates a new PathCommand. */
  public FollowPathCommand PathCommand() {

    return new FollowPathHolonomic(
        m_path,
        m_drive::getPose,
        m_drive::getCurrentspeeds, // MUST BE ROBOT RELATIVE
        m_drive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        DriveConstants.holonomicConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
              return true;
            }
          }
          return false;
        },
        m_drive // Reference to this subsystem to set requirements
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_path.getPreviewStartingHolonomicPose());

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        m_drive.resetOdometry(mirrorPose(m_path.getPreviewStartingHolonomicPose())); // TODO: only reset odometry once
      }
    }
    m_pathPlannerCommand.initialize();
  }

  public Pose2d mirrorPose(Pose2d inputPose2d) {
    Pose2d output = new Pose2d(54 * 12 * 0.0254 - inputPose2d.getX(), inputPose2d.getY(),
        new Rotation2d(Math.PI).minus(inputPose2d.getRotation()));
    return output;
  }
  // public Optional<Rotation2d> getRotationTargetOverride() {
  // // Some condition that should decide if we want to override rotation
  // if(Limelight.hasGamePieceTarget()) {
  // // Return an optional containing the rotation override (this should be a
  // field relative rotation)
  // return Optional.of(Limelight.getRobotToGamePieceRotation());
  // } else {
  // // return an empty optional when we don't want to override the path's
  // rotation
  // return Optional.empty();
  // }
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pathPlannerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pathPlannerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pathPlannerCommand.isFinished();
  }
}
