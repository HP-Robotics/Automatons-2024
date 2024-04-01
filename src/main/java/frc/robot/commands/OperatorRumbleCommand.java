// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OperatorRumbleCommand extends Command {
  PivotSubsystem m_pivot;
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;
  ShooterSubsystem m_shooter;
  CommandJoystick m_joystick;

  /** Creates a new ShooterCommand. */
  public OperatorRumbleCommand(PivotSubsystem pivot, DriveSubsystem drive, LimelightSubsystem limelight,
      ShooterSubsystem shooter, CommandJoystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies
    m_pivot = pivot;
    m_drive = drive;
    m_limelight = limelight;
    m_shooter = shooter;
    m_joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_pivot.atPosition() && m_drive.pointedTowardsAngle() && m_shooter.atSpeed()) {
      m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}