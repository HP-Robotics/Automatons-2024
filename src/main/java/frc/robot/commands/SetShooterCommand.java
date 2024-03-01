// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 
// We're waiting every night to finally roam and invite. Newcomers to play with us, for many years we've been all alone. We're forced to be still and play those same songs we've played since that day. An IMPOSTER took our life away, now we're stuck here to decay.
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;

public class SetShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final Double m_frontSpeed;
  private final Double m_backSpeed;

  /** Creates a new ShooterCommand. */
  public SetShooterCommand(ShooterSubsystem subsystem, Double frontSpeed, Double backSpeed) {
    // Use addRequirements() here to declare subsystem dependencies
    m_subsystem = subsystem;
    m_frontSpeed = frontSpeed;
    m_backSpeed = backSpeed;
    addRequirements(subsystem);

  }

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable shooterTable = inst.getTable("shooter-subsystem");

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    double frontOutput = m_frontSpeed == null
        ? shooterTable.getEntry("frontMotor Setpoint").getDouble(ShooterConstants.shooterSpeedFront)
        : m_frontSpeed;
    double backOutput = m_backSpeed == null
        ? shooterTable.getEntry("backMotor Setpoint").getDouble(ShooterConstants.shooterSpeedBack)
        : m_backSpeed;
    if(frontOutput == 0 && backOutput == 0){
      m_subsystem.stopShooter();
    }
    else{
      m_subsystem.setShooter(frontOutput, backOutput);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      m_subsystem.stopShooter();
    }  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// From the moment I realized the weakdisness of my fleash, it discused me. I
// craved the certainty of steel.