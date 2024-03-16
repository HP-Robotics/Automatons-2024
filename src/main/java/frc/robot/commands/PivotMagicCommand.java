// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PivotMagicCommand extends Command {
  private final PivotSubsystem m_subsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private Pose2d m_targetAprilTag;
  private CommandJoystick m_joystick;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable pivotTable = inst.getTable("pivot-table");

  /** Creates a new pivotMagicCommand. */
  public PivotMagicCommand(PivotSubsystem subsystem, LimelightSubsystem limelightSubsystem, CommandJoystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_limelightSubsystem = limelightSubsystem;
    addRequirements(subsystem);
    m_joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        m_targetAprilTag = LimelightConstants.aprilTagList[7];
      } else {
        m_targetAprilTag = LimelightConstants.aprilTagList[4];
      }
    } else {
      m_targetAprilTag = LimelightConstants.aprilTagList[7];
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelightSubsystem.sawAprilTag) {
      if (m_limelightSubsystem.getDistanceTo(m_limelightSubsystem.m_visionPose2d, m_targetAprilTag) < 3.6) {
        m_subsystem.setPosition(m_subsystem.getMagicAngle(
          m_limelightSubsystem.getDistanceTo(m_limelightSubsystem.m_visionPose2d, m_targetAprilTag)));
          m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0.2);
      }
      else {
        m_joystick.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      pivotTable.putValue(
          "magicEncoderValue", NetworkTableValue.makeDouble(m_subsystem.getMagicAngle(
              m_limelightSubsystem.getDistanceTo(m_limelightSubsystem.m_visionPose2d, m_targetAprilTag))));
    }
    else {
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
