// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TriggerConstants;
public class TriggerCommand extends Command {
   private final TriggerSubsystem m_subsystem;
    boolean m_ignoreBeamBreak;
    boolean m_currentbeambreak = false;
    boolean pastBeamBroken = false;
    IntakeSubsystem m_intakeSubsystem;

  /** Creates a new ShooterCommand. */
  public TriggerCommand(TriggerSubsystem subsystem, Boolean ignoreBeamBreak, IntakeSubsystem intakesubsystem) {
    // Use addRequirements() here to declare subsystem dependencies
    m_subsystem = subsystem;
    m_intakeSubsystem = intakesubsystem;
    addRequirements(subsystem);
    m_ignoreBeamBreak = ignoreBeamBreak;
  }

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable triggerTable = inst.getTable("trigger-subsystem");

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double output = triggerTable.getEntry("Trigger Setpoint").getDouble(TriggerConstants.triggerSpeed);
    m_subsystem.setTrigger(output); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pastBeamBroken = m_currentbeambreak;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopTrigger();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_currentbeambreak = m_intakeSubsystem.m_beambreak.beamBroken();
    if (!m_ignoreBeamBreak) {
      if (pastBeamBroken != m_currentbeambreak && pastBeamBroken) {
        System.out.println("stopped");
        return true;
      }
    }
    return false;
  }
}

// From the moment I realized the weakdisness of my fleash, it discused me. I craved the certainty of steel. 
