// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.TriangleInterpolator;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PresetShotForPathCommand extends InstantCommand {
  private PivotSubsystem m_pivotSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private TriangleInterpolator m_triangleInterpolator;
  private PathPlannerPath m_path;

  /** Creates a new PresetPivotForPathCommand. */
  public PresetShotForPathCommand(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem,
      TriangleInterpolator triangleInterpolator, String pathName) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_triangleInterpolator = triangleInterpolator;
    m_pivotSubsystem = pivotSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_path = PathPlannerPath.fromPathFile(pathName);

    //addRequirements(pivotSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPoint endPoint = m_path.getPoint(m_path.numPoints() - 1);
    Pose2d pose = new Pose2d(endPoint.position, new Rotation2d());

    Optional<double[]> shotValues = m_triangleInterpolator.getTriangulatedOutput(pose);
    if (shotValues.isEmpty()) {
      return;
    }
    // Set these values
    m_shooterSubsystem.setShooter(shotValues.get()[0], shotValues.get()[1]);
    m_pivotSubsystem.setPosition(shotValues.get()[2]);
  }
}
