// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_subsystem;
    Boolean pastBeamBroken = false;
    Boolean currentbeambreak = false;
  /** Creates a new IntakeCommand. */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("intake-table");
  public IntakeCommand(IntakeSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double output = intakeTable.getEntry("Intake Setpoint").getDouble(IntakeConstants.intakeSpeed);
    m_subsystem.runIntake(output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pastBeamBroken = currentbeambreak;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.runIntake(0.0); // Turn intake off; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentbeambreak = m_subsystem.m_beambreak.beamBroken();
    if (pastBeamBroken != currentbeambreak && pastBeamBroken) {
      return true;
    }
    return false;
  }
}
