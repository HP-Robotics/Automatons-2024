// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.BeamBreak;
import frc.robot.Constants.TriggerConstants;

public class TriggerStatesCommand extends Command {
  private final TriggerSubsystem m_subsystem;
  private BeamBreak m_beamBreak;
  /** Creates a new IntakeCommand. */
  NetworkTableInstance inst = NetworkTableInstance.getDefault();  // JPW TODO m_
  NetworkTable triggerTable = inst.getTable("trigger-subsystem");  // JPW TODO m_

  public TriggerStatesCommand(TriggerSubsystem subsystem, BeamBreak beambreak) {
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
      m_subsystem.beambreakCount = 0;
      m_subsystem.m_isLoaded = true;
    }
    if (m_subsystem.m_isYucking) {
      m_subsystem.setTrigger(TriggerConstants.yuckSpeed);
    } else if (m_subsystem.m_isFiring && m_subsystem.beambreakCount > 2) {
      m_subsystem.setTrigger(triggerTable.getEntry("Trigger Setpoint").getDouble(TriggerConstants.triggerSpeed));
    } else if (m_subsystem.m_isLoaded) {
      m_subsystem.beambreakCount += 1;
      m_subsystem.m_triggerMotor.setControl(new NeutralOut());
    } else if (m_subsystem.m_isIntaking) {
      m_subsystem.setTrigger(triggerTable.getEntry("Trigger Setpoint").getDouble(TriggerConstants.triggerSpeed));//divide this by two if triger toasters 
    } else {
      m_subsystem.m_triggerMotor.setControl(new NeutralOut());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setTrigger(0.0); // Turn intake off;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
