// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.Optional;

public class DrivePointedToNoteCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final CommandJoystick m_joystick;

  /** Creates a new IntakeCommand. */
  public DrivePointedToNoteCommand(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem,
      CommandJoystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_joystick = joystick;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Double> angle = m_limelightSubsystem.getNoteTX();
    if (angle.isPresent() && Math.abs(angle.get()) > LimelightConstants.allowableNoteAngleError) {
      m_driveSubsystem.drivePointedTowardsAngle(m_joystick,
          new Rotation2d(Math
              .toRadians(-angle.get())).plus(m_driveSubsystem.getPose().getRotation())); // TODO: Add intaking
    } else {
      m_driveSubsystem.driveWithJoystick(m_joystick);
    }
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
