// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 
// We're waiting every night to finally roam and invite. Newcomers to play with us, for many years we've been all alone. We're forced to be still and play those same songs we've played since that day. An IMPOSTER took our life away, now we're stuck here to decay.
package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.TriangleInterpolator;
import frc.robot.Constants.ShooterConstants;

public class SetShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final Double m_leftSpeed;
  private final Double m_rightSpeed;
  private TriangleInterpolator m_magicTriangles = null;
  private TriangleInterpolator m_feederInterpolator = null;
  private PoseEstimatorSubsystem m_poseEstimator = null;

  /**
   * Creates a new ShooterCommand with left and right speeds. *
   * 
   * @param subsystem
   * @param leftSpeed
   * @param rightSpeed
   */
  public SetShooterCommand(ShooterSubsystem subsystem, Double leftSpeed, Double rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies
    m_subsystem = subsystem;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    addRequirements(subsystem);
  }

  /**
   * Gets speeds from NetworkTables
   * 
   * @param subsystem
   */
  public SetShooterCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    m_leftSpeed = null;
    m_rightSpeed = null;
    addRequirements(subsystem);
  }

  /**
   * Uses the magic for shooter speed *
   * 
   * @param subsystem
   * @param poseEstimator
   * @param triangle
   */
  public SetShooterCommand(ShooterSubsystem subsystem, PoseEstimatorSubsystem poseEstimator,
      TriangleInterpolator triangle, TriangleInterpolator feederInterpolator) {
    this(subsystem);
    m_poseEstimator = poseEstimator;
    m_magicTriangles = triangle;
    m_feederInterpolator = feederInterpolator;
  }

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable shooterTable = inst.getTable("shooter-subsystem");

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean usingNetworkTables = true;
    double leftOutput = 0;
    double rightOutput = 0;
    Pose2d currentPose = m_poseEstimator == null
        ? null
        : m_poseEstimator.getAlliancePose(); // TODO: take a look at this
    if (currentPose != null) {
      Optional<double[]> interpolatorData = m_magicTriangles.getTriangulatedOutput(currentPose);
      if (interpolatorData.isEmpty()) {
        interpolatorData = m_feederInterpolator.getTriangulatedOutput(currentPose);
      }

      if (interpolatorData.isPresent()) {
        usingNetworkTables = false;
        // System.out.println("Recived Data");
        leftOutput = interpolatorData.get()[0];
        rightOutput = interpolatorData.get()[1];
      }
    } else if (m_leftSpeed != null && m_rightSpeed != null) {
      usingNetworkTables = false;
      leftOutput = m_leftSpeed;
      rightOutput = m_rightSpeed;
    }
    if (usingNetworkTables) {
      leftOutput = shooterTable.getEntry("leftMotor Setpoint").getDouble(ShooterConstants.shooterSpeedLeft);
      rightOutput = shooterTable.getEntry("rightMotor Setpoint").getDouble(ShooterConstants.shooterSpeedRight);
    }
    m_subsystem.setShooter(leftOutput, rightOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftOutput = 0;
    double rightOutput = 0;
    Pose2d currentPose = m_poseEstimator == null
        ? null
        : m_poseEstimator.getAlliancePose(); // TODO: look at this
    if (currentPose != null) {
      Optional<double[]> interpolatorData = m_magicTriangles.getTriangulatedOutput(currentPose);
      if (interpolatorData.isEmpty()) {
        interpolatorData = m_feederInterpolator.getTriangulatedOutput(currentPose);
      }
      
      if (interpolatorData.isPresent()) {
        // System.out.println("Recived Data");
        leftOutput = interpolatorData.get()[0];
        rightOutput = interpolatorData.get()[1];
      } else {
        leftOutput = shooterTable.getEntry("leftMotor Setpoint").getDouble(ShooterConstants.shooterSpeedLeft);
        rightOutput = shooterTable.getEntry("rightMotor Setpoint").getDouble(ShooterConstants.shooterSpeedRight);
      }
      m_subsystem.setShooter(leftOutput, rightOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
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