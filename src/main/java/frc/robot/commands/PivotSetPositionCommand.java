// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotSetPositionCommand extends Command {
  private final PivotSubsystem m_subsystem;
  double m_position;
  /** Creates a new pivotManualCommand. */
  public PivotSetPositionCommand(PivotSubsystem subsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_position = position;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.movePivot(m_position, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.movePivot(0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}