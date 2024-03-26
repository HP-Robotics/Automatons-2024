// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 
// We're waiting every night to finally roam and invite. Newcomers to play with us, for many years we've been all alone. We're forced to be still and play those same songs we've played since that day. An IMPOSTER took our life away, now we're stuck here to decay.
package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.TriangleInterpolator;
import frc.robot.Constants.ShooterConstants;

public class SetShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final Double m_frontSpeed;
  private final Double m_backSpeed;
  private TriangleInterpolator m_magicTriangles = null;
  private PoseEstimatorSubsystem m_poseEstimator = null;

  /** Creates a new ShooterCommand. */
  public SetShooterCommand(ShooterSubsystem subsystem, Double frontSpeed, Double backSpeed) {
    // Use addRequirements() here to declare subsystem dependencies
    m_subsystem = subsystem;
    m_frontSpeed = frontSpeed;
    m_backSpeed = backSpeed;
    addRequirements(subsystem);

  }

  public SetShooterCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    m_frontSpeed = null;
    m_backSpeed = null;
    addRequirements(subsystem);
  }

  public SetShooterCommand(ShooterSubsystem subsystem, PoseEstimatorSubsystem poseEstimator,
      TriangleInterpolator triangle) {
    this(subsystem);
    m_poseEstimator = poseEstimator;
    m_magicTriangles = triangle;
  }

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable shooterTable = inst.getTable("shooter-subsystem");

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double frontOutput = 0;
    double backOutput = 0;
    Pose2d currentPose = m_poseEstimator == null
        ? null
        : m_poseEstimator.getPose();
    if (currentPose == null) {
      frontOutput = m_frontSpeed == null
          ? shooterTable.getEntry("frontMotor Setpoint").getDouble(ShooterConstants.shooterSpeedFront)
          : m_frontSpeed;
      backOutput = m_backSpeed == null
          ? shooterTable.getEntry("backMotor Setpoint").getDouble(ShooterConstants.shooterSpeedBack)
          : m_backSpeed;
    } else {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        currentPose = GeometryUtil.flipFieldPose(currentPose);
      }
      Optional<double[]> triangleData = m_magicTriangles.getTriangulatedOutput(currentPose);
      if (triangleData.isPresent()) {
        // System.out.println("Recived Data");
        frontOutput = triangleData.get()[0];
        backOutput = triangleData.get()[1];
      }
      else{
        frontOutput = shooterTable.getEntry("frontMotor Setpoint").getDouble(ShooterConstants.shooterSpeedFront);
        backOutput = shooterTable.getEntry("backMotor Setpoint").getDouble(ShooterConstants.shooterSpeedBack);
      }
    }
    if (frontOutput == 0 && backOutput == 0) {
      m_subsystem.stopShooter();
    } else {
      m_subsystem.setShooter(frontOutput, backOutput);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double frontOutput = 0;
    double backOutput = 0;
    Pose2d currentPose = m_poseEstimator == null
        ? null
        : m_poseEstimator.getAlliancePose(); // TODO: look at this
    if (currentPose != null) {
      Optional<double[]> triangleData = m_magicTriangles.getTriangulatedOutput(currentPose);
      if (triangleData.isPresent()) {
        // System.out.println("Recived Data");
        frontOutput = triangleData.get()[0];
        backOutput = triangleData.get()[1];
      }
      else{
        frontOutput = shooterTable.getEntry("frontMotor Setpoint").getDouble(ShooterConstants.shooterSpeedFront);
        backOutput = shooterTable.getEntry("backMotor Setpoint").getDouble(ShooterConstants.shooterSpeedBack);
      }
      if (frontOutput == 0 && backOutput == 0) {
      m_subsystem.stopShooter();
      } 
      else {
      m_subsystem.setShooter(frontOutput, backOutput);
      }
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