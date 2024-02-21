// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.BeamBreak;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TriggerConstants;

public class TriggerStatesCommand extends Command {
    private final TriggerSubsystem m_subsystem;
    private BeamBreak m_beamBreak;
  /** Creates a new IntakeCommand. */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable triggerTable = inst.getTable("trigger-subsystem");
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
    if (m_subsystem.triggerYuck) {
      m_subsystem.setTrigger(TriggerConstants.yuckSpeed);
    }
    else if (m_subsystem.triggerFire ) {
      m_subsystem.setTrigger(triggerTable.getEntry("Trigger Setpoint").getDouble(TriggerConstants.triggerSpeed));
    }
    else if (m_subsystem.beambreakState) {
      m_subsystem.m_triggerMotor.setControl(new NeutralOut());
    }
    else if (m_subsystem.triggerOn) {
      m_subsystem.setTrigger(triggerTable.getEntry("Trigger Setpoint").getDouble(TriggerConstants.triggerSpeed));
    }
    else {
      m_subsystem.m_triggerMotor.setControl(new NeutralOut());
    }
    if (m_subsystem.beambreakState && !m_beamBreak.beamBroken()) {
      m_subsystem.beambreakState = false;
    }
    if (!m_subsystem.beambreakState && m_beamBreak.beamBroken()) {
      m_subsystem.beambreakState = true;
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
