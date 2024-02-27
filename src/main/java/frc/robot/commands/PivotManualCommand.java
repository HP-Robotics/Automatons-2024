// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotManualCommand extends Command {
  private final PivotSubsystem m_subsystem;
  double m_speed;

  /** Creates a new pivotManualCommand. */
  public PivotManualCommand(PivotSubsystem subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_speed = speed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_subsystem.getUsePID()) {
      m_subsystem.setSpeed(m_speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.getUsePID()) {
      m_subsystem.setPosition(m_subsystem.getCurrentPosition() + m_speed * PivotConstants.setpointChangeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_subsystem.getUsePID()) {
      m_subsystem.setPosition(m_subsystem.getCurrentPosition());
    } else {
      m_subsystem.setSpeed(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
