// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.BeamBreak;
import frc.robot.Constants.IntakeConstants;

public class IntakeStatesCommand extends Command {
  private final IntakeSubsystem m_subsystem;
  private BeamBreak m_beamBreak;
  /** Creates a new IntakeCommand. */
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = inst.getTable("intake-table");

  public IntakeStatesCommand(IntakeSubsystem subsystem, BeamBreak beambreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_beamBreak = beambreak;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.m_isLoaded && !m_beamBreak.beamBroken()) {
      m_subsystem.m_isLoaded = false;
    }
    if (!m_subsystem.m_isLoaded && m_beamBreak.beamBroken()) {
      m_subsystem.m_isLoaded = true;
    }
    if (m_subsystem.m_isYucking) {
      m_subsystem.runIntake(IntakeConstants.intakeSpeed,
          IntakeConstants.vanguardSpeedSide, IntakeConstants.vanguardSpeedFront);
    } else if (m_subsystem.m_isFiring) {
      m_subsystem.runIntake(IntakeConstants.intakeSpeed, 0, 0);
    } else if (m_subsystem.m_isLoaded) {
      m_subsystem.runIntake(0, 0, 0);
    } else if (m_subsystem.m_isIntaking) {
      m_subsystem.runIntake(IntakeConstants.intakeSpeed,
          IntakeConstants.vanguardSpeedSide, IntakeConstants.vanguardSpeedFront);
          
    } else {
      m_subsystem.runIntake(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.runIntake(0.0, 0.0, 0.0); // Turn intake off;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
