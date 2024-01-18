// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveModule;
import frc.robot.subsystems.DriveSubsystem;

public class DriveParkingBrakeCommand extends Command {
  /** Creates a new DriveParkingBrakeCommand. */
  DriveSubsystem m_subsystem;

  public DriveParkingBrakeCommand(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState fl = new SwerveModuleState();
    SwerveModuleState fr = new SwerveModuleState();
    SwerveModuleState br = new SwerveModuleState();
    SwerveModuleState bl = new SwerveModuleState();
    fl.angle = new Rotation2d(Math.PI / 4);
    fr.angle = new Rotation2d(Math.PI / -4);
    br.angle = new Rotation2d(Math.PI / 4);
    bl.angle = new Rotation2d(Math.PI / -4);
    SwerveModuleState[] swerveModuleStates = { fl, fr, br, bl };
    m_subsystem.setModuleStates(swerveModuleStates);
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
