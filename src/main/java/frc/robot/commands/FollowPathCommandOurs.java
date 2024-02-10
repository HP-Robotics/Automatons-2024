// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
        m_drive::getPose, // Robot pose supplier
        m_drive::getCurrentspeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_drive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          return false;
        },
        m_drive // Reference to this subsystem to set requirements
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_path.getPreviewStartingHolonomicPose());
    m_pathPlannerCommand.initialize();
    
  }
// public Optional<Rotation2d> getRotationTargetOverride() {
//       // Some condition that should decide if we want to override rotation
//       if(Limelight.hasGamePieceTarget()) {
//           // Return an optional containing the rotation override (this should be a field relative rotation)
//           return Optional.of(Limelight.getRobotToGamePieceRotation());
//       } else {
//           // return an empty optional when we don't want to override the path's rotation
//           return Optional.empty();
//       }
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
