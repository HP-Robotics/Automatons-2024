package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TriangleInterpolator;
import frc.robot.Constants.SnuffilatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SnuffilatorSubsystem;
import frc.robot.subsystems.TriggerSubsystem;

public class CommandBlocks {
  DriveSubsystem m_driveSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  TriggerSubsystem m_triggerSubsystem;
  PivotSubsystem m_pivotSubsystem;
  SnuffilatorSubsystem m_snuffilatorSubsystem;
  LimelightSubsystem m_limelightSubsystem;
  PoseEstimatorSubsystem m_poseEstimator;
  TriangleInterpolator m_triangleInterpolator; 
  TriangleInterpolator m_feederInterpolator;

  public CommandBlocks(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem,
      SnuffilatorSubsystem snuffilatorSubsystem, LimelightSubsystem limelightSubsystem,
      PoseEstimatorSubsystem poseEstimator, TriangleInterpolator triangleInterpolator, TriangleInterpolator feederInterpolator) {
    m_driveSubsystem = driveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_triggerSubsystem = triggerSubsystem;
    m_snuffilatorSubsystem = snuffilatorSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_poseEstimator = poseEstimator;
    m_triangleInterpolator = triangleInterpolator;
    m_feederInterpolator = feederInterpolator;
  }

  public Command fireGamePieceCommand() {
    if (m_intakeSubsystem == null || m_triggerSubsystem == null || m_shooterSubsystem == null
        || m_pivotSubsystem == null) {
      return new WaitCommand(0);
    }
    return new ParallelCommandGroup(
        new InstantCommand(
            () -> new SetShooterCommand(m_shooterSubsystem, m_poseEstimator, m_triangleInterpolator, m_feederInterpolator).schedule()),
        new InstantCommand(
            () -> new PivotMagicCommand(m_pivotSubsystem, m_limelightSubsystem, m_triangleInterpolator, m_feederInterpolator, m_poseEstimator)
                .schedule()))
        .andThen(
            new ParallelDeadlineGroup(
                new WaitCommand(1).until(() -> {
                  /* System.out.println("wait for shooter"); */return m_shooterSubsystem.atSpeed()
                      && m_pivotSubsystem.atPosition() && m_triggerSubsystem.m_isLoaded;
                }).andThen(fireButtonHold().until(() -> {
                  /* System.out.println("waiting for fire"); */return !m_triggerSubsystem.m_isLoaded;
                })),
                intakeButtonHold()
            // new InstantCommand(() -> {System.out.println("firing game piece");})
            ));
  }

  public Command fireGamePieceCommand(double pivotAngle, double leftSpeed, double rightSpeed) {
    if (m_intakeSubsystem == null || m_triggerSubsystem == null || m_shooterSubsystem == null
        || m_pivotSubsystem == null) {
      return new WaitCommand(0);
    }
    return new ParallelCommandGroup(
        new InstantCommand(
            () -> m_shooterSubsystem.setShooter(leftSpeed, rightSpeed)),
        new InstantCommand(() -> m_pivotSubsystem.setPosition(pivotAngle)))
        .andThen(
            new ParallelDeadlineGroup(
                new WaitCommand(1).until(() -> {
                  /* System.out.println("wait for shooter"); */return m_shooterSubsystem.atSpeed()
                      && m_pivotSubsystem.atPosition() && m_triggerSubsystem.m_isLoaded;
                }).andThen(fireButtonHold().until(() -> {
                  /* System.out.println("waiting for fire"); */return !m_triggerSubsystem.m_isLoaded;
                })),
                intakeButtonHold()
            // new InstantCommand(() -> {System.out.println("firing game piece");})
            ));
  }

  public Command intakeButtonHold() {
    if (m_intakeSubsystem == null || m_triggerSubsystem == null) {
      return new WaitCommand(0);
    }
    return new ParallelCommandGroup(
        new StartEndCommand(m_triggerSubsystem::intakeButtonPressed, m_triggerSubsystem::intakeButtonReleased),
        new StartEndCommand(m_intakeSubsystem::intakeButtonPressed, m_intakeSubsystem::intakeButtonReleased));
  }

  public Command startIntaking() {
    if (m_intakeSubsystem == null || m_triggerSubsystem == null) {
      return new WaitCommand(0);
    }
    return new ParallelCommandGroup(
        new InstantCommand(m_intakeSubsystem::intakeButtonPressed),
        new InstantCommand(m_triggerSubsystem::intakeButtonPressed));
  }

  public Command stopIntaking() {
    if (m_intakeSubsystem == null || m_triggerSubsystem == null) {
      return new WaitCommand(0);
    }
    return new ParallelCommandGroup(
        new InstantCommand(m_intakeSubsystem::intakeButtonReleased),
        new InstantCommand(m_triggerSubsystem::intakeButtonReleased));
  }

  public Command fireButtonHold() {
    if (m_intakeSubsystem == null || m_triggerSubsystem == null) {
      return new WaitCommand(0);
    }
    return new ParallelCommandGroup(
        new StartEndCommand(m_triggerSubsystem::fireButtonPressed, m_triggerSubsystem::fireButtonReleased),
        new StartEndCommand(m_intakeSubsystem::fireButtonPressed, m_intakeSubsystem::fireButtonReleased));
  }

  public Command yuckButtonHold() {
    if (m_intakeSubsystem == null || m_triggerSubsystem == null) {
      return new WaitCommand(0);
    }
    return new ParallelCommandGroup(
        new StartEndCommand(m_triggerSubsystem::yuckButtonPressed, m_triggerSubsystem::yuckButtonReleased),
        new StartEndCommand(m_intakeSubsystem::yuckButtonPressed, m_intakeSubsystem::yuckButtonReleased));
  }

  public Command moveSnuffilator(boolean goingOut) {
    if (goingOut) {
      return new InstantCommand(() -> {
        m_snuffilatorSubsystem.move(SnuffilatorConstants.snuffilatorOutSpeed);
      });
    } else {
      return new InstantCommand(() -> {
        m_snuffilatorSubsystem.move(-SnuffilatorConstants.snuffilatorInSpeed);
      });
    }
  }

  public Command followPathWithPresetShot(String pathName, boolean useLimelight, boolean resetPosition) {
    if (m_driveSubsystem == null || m_triangleInterpolator == null || m_shooterSubsystem == null
        || m_pivotSubsystem == null) {
      return new ParallelCommandGroup(
      new WaitCommand(0)
      );
    }
    if (useLimelight) {
      return new ParallelDeadlineGroup(
          new FollowPathCommandOurs(m_driveSubsystem, m_limelightSubsystem, pathName, resetPosition),
          new PresetShotForPathCommand(m_pivotSubsystem, m_shooterSubsystem, m_triangleInterpolator, pathName));
    } else {
      return new ParallelDeadlineGroup(
          new FollowPathCommandOurs(m_driveSubsystem, pathName, resetPosition),
          new PresetShotForPathCommand(m_pivotSubsystem, m_shooterSubsystem, m_triangleInterpolator, pathName));
    }
  }

  public Command followPathWithPresetShot(String pathName, boolean useLimelight) {
    return followPathWithPresetShot(pathName, useLimelight, false);
  }

  public Command instantSetPivot(double pivotAngle) {
    if (m_pivotSubsystem == null) {
      return null;
    }
    return new InstantCommand(() -> m_pivotSubsystem.setPosition(pivotAngle));
  }
}
