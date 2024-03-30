// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPoseCommand extends Command {
  DriveSubsystem m_driveSubsystem;
  String m_pathName;
  PathPlannerPath m_path;

  public PathfindThenFollowPathHolonomic m_pathPlannerCommand;
  public PPHolonomicDriveController m_HolonomicDriveController;

  public DriveToPoseCommand(DriveSubsystem driveSubsystem, String pathName) {
    m_driveSubsystem = driveSubsystem;
    m_pathName = pathName;

    m_path = PathPlannerPath.fromPathFile(pathName);
    m_pathPlannerCommand = PathCommand();
    addRequirements(driveSubsystem);
  }

  /** Creates a new PathCommand. */
  public PathfindThenFollowPathHolonomic PathCommand() {

    return new PathfindThenFollowPathHolonomic(
      m_path, 
      new PathConstraints(AutoConstants.kMaxAutoVelocity, 
      AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
      AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
      AutoConstants.kMaxAngularAcceleration), 
      m_driveSubsystem::getPose, 
      m_driveSubsystem::getCurrentspeeds, 
      m_driveSubsystem::driveRobotRelative, 
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
      m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_pathPlannerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pathPlannerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pathPlannerCommand.end(interrupted);
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_pathPlannerCommand.isFinished();
  }
}
