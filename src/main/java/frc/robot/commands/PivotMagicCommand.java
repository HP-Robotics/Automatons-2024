// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PivotMagicCommand extends Command {
  private final PivotSubsystem m_subsystem;
  private final LimelightSubsystem m_limelightSubsystem;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable pivotTable = inst.getTable("pivot-table");

  /** Creates a new pivotMagicCommand. */
  public PivotMagicCommand(PivotSubsystem subsystem, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_limelightSubsystem = limelightSubsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelightSubsystem.sawAprilTag == 1) {
      m_subsystem.setPosition(m_subsystem.getMagicAngle(m_limelightSubsystem.getDistanceTo(m_limelightSubsystem.m_visionPose2d, LimelightConstants.aprilTag7)));
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
