// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_subsystem;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double output = SmartDashboard.getNumber("Intake Speed", IntakeConstants.intakeSpeed);
    m_subsystem.runIntake(output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.runIntake(0.0); // Turn intake off; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
