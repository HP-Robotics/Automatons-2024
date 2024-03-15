// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TriggerSubsystem;

public class DriveToNoteCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final TriggerSubsystem m_triggerSubsystem;
  private final CommandJoystick m_joystick;
  Boolean pastBeamBroken = false;
  Boolean currentbeambreak = false;
  Boolean m_seenNote = false;
  Rotation2d m_noteHeading;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = inst.getTable("intake-table");

  public DriveToNoteCommand(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem,
      IntakeSubsystem intakeSubsystem, TriggerSubsystem triggerSubsystem, CommandJoystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_triggerSubsystem = triggerSubsystem;
    m_joystick = joystick;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intakeButtonPressed();
    m_triggerSubsystem.intakeButtonPressed();
    m_seenNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Double> angle = m_limelightSubsystem.getNoteTX();
    if (angle.isPresent()) {
      m_seenNote = true;
      m_noteHeading = new Rotation2d(Math.toRadians(-angle.get())).plus(m_driveSubsystem.getPose().getRotation());
    }
    if (m_seenNote == true) {
      m_driveSubsystem.driveToNote(DriveConstants.driveToNoteSpeed, m_noteHeading);
    } else {
      m_driveSubsystem.driveWithJoystick(m_joystick);
    }
    pastBeamBroken = currentbeambreak;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true);
    if (interrupted) {
      m_intakeSubsystem.intakeButtonReleased();
      m_triggerSubsystem.intakeButtonReleased();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentbeambreak = m_intakeSubsystem.m_beambreak.beamBroken();
    if (pastBeamBroken != currentbeambreak && !pastBeamBroken) {
      return true;
    }
    return false;
  }
}
