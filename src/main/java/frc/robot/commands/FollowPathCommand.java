// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowPathCommand extends Command {
  DriveSubsystem m_drive;
  String m_pathName; 
  PathPlannerPath m_path;
  

  public Command m_pathPlannerCommand;

  public FollowPathCommand(DriveSubsystem Drive, String PathName) {
    m_drive = Drive;
    m_pathName = PathName;

    m_path = PathPlannerPath.fromPathFile(PathName);
    m_pathPlannerCommand = PathCommand();
    addRequirements(Drive);
  }

  /** Creates a new PathCommand. */
  public Command PathCommand() {

    return new FollowPathHolonomic(
        m_path,
        m_drive::getPose, 
        m_drive::getCurrentspeeds, // MUST BE ROBOT RELATIVE
        m_drive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // TODO move to Constants
            new PIDConstants(5.0, 0.0, 0.0), 
            new PIDConstants(5.0, 0.0, 0.0), 
            4.5,
            0.4, // Distance from robot center to furthest module.
            new ReplanningConfig()
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
