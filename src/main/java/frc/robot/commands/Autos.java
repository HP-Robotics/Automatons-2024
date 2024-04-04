// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TriangleInterpolator;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }

  public static Command ShootPreloadFarAway(CommandBlocks commandBlocks, DriveSubsystem drive,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, PivotSubsystem pivotSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    /*
     * There was a point where a version of the call to 'getDistanceTo' would cause
     * the rio to apparently
     * start the robot over and over again. It was cool; you'd get many printed
     * messages of the robot
     * starting itself. That occurred when trying to make the method static and use
     * it.
     * It vanished by itself in a not entirely logical way. Our best guess was that
     * compile artifacts
     * needed to be cleaned more thoroughly than gradle could detect. But this
     * comment is here to remind us
     * if we ever see that error return. -- Mentor Jeremy White
     */
    return new SequentialCommandGroup(
        new FollowPathCommandOurs(drive, "Shoot Preload Far Away", true),
        commandBlocks.fireGamePieceCommand(PivotConstants.preloadFarAwayPosition, ShooterConstants.preloadSpeedLeft, ShooterConstants.preloadSpeedRight));
  }

  public static Command GrandTheftAuto(DriveSubsystem drive) {
    if (!SubsystemConstants.useDrive) {
      return null;
    }
    return new SequentialCommandGroup(
        new FollowPathCommandOurs(drive, "Grand Theft Auto Part 1", true));
  }

  public static Command FiveMeterTest(DriveSubsystem drive) {
    if (!SubsystemConstants.useDrive) {
      return null;
    } else {
      return new FollowPathCommandOurs(drive, "Test Path 5 Meters", true);
    }
  }

  public static Command OnlyShoot(CommandBlocks commandBlocks, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useShooter) {
      return new WaitCommand(0);
    }
    return commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition, ShooterConstants.preloadSpeedLeft, ShooterConstants.preloadSpeedRight);
  }

  public static Command AmpCenter4Piece(CommandBlocks commandBlocks, DriveSubsystem drive,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, TriggerSubsystem triggerSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem, TriangleInterpolator triangles) {
    if (!SubsystemConstants.useDrive || !SubsystemConstants.useIntake || !SubsystemConstants.useShooter
        || !SubsystemConstants.useLimelight || !SubsystemConstants.useTrigger) {
      return null;
    }
    return new SequentialCommandGroup(
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition, ShooterConstants.preloadSpeedLeft, ShooterConstants.preloadSpeedRight).withTimeout(1.5),
        commandBlocks.instantSetPivot(PivotConstants.podiumPosition),
        new FollowPathCommandOurs(drive, limelightSubsystem, "Amp Center 4 Piece Part 1", true),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(1),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5),
        new FollowPathCommandOurs(drive, limelightSubsystem, "Amp Center 4 Piece Part 2"),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(2),
        commandBlocks.followPathWithPresetShot("Amp Center 4 Piece Part 3", false),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5),
        new FollowPathCommandOurs(drive, limelightSubsystem, "Amp Center 4 Piece Part 4"),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(2),
        commandBlocks.followPathWithPresetShot("Amp Center 4 Piece Part 5", false),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5),
        new WaitCommand(1));
  }

  public static Command MiddleAllianceFourPiece(CommandBlocks commandBlocks, DriveSubsystem drive,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, TriggerSubsystem triggerSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem, TriangleInterpolator triangles) {
    if (!SubsystemConstants.useDrive || !SubsystemConstants.useIntake || !SubsystemConstants.useShooter
        || !SubsystemConstants.useLimelight || !SubsystemConstants.useTrigger) {
      return null;
    }
    return new SequentialCommandGroup(
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition, ShooterConstants.preloadSpeedLeft, ShooterConstants.preloadSpeedRight).withTimeout(1.5),
        // commandBlocks.followPathWithPresetShot("Middle Alliance 4 Piece Part 1", true, true),
        new FollowPathCommandOurs(drive, limelightSubsystem, "Middle Alliance 4 Piece Part 1", true),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(1),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5), // TODO: Add magic to these
        new FollowPathCommandOurs(drive, limelightSubsystem, "Middle Alliance 4 Piece Part 2"),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(1),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5), // TODO: Add magic to these
        new FollowPathCommandOurs(drive, limelightSubsystem, "Middle Alliance 4 Piece Part 3"),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(1),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5), // TODO: Add magic to these
        new FollowPathCommandOurs(drive, limelightSubsystem, "Middle Alliance 5 Piece Part 4"),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(0.75),
        commandBlocks.followPathWithPresetShot("Middle Alliance 5 Piece Part 5", false),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand());
  }

  public static Command NoteCancelTest(CommandBlocks commandBlocks, DriveSubsystem drive,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, TriggerSubsystem triggerSubsystem) {
    if (!SubsystemConstants.useDrive || !SubsystemConstants.useIntake || !SubsystemConstants.useShooter
        || !SubsystemConstants.useLimelight || !SubsystemConstants.useTrigger) {
      return null;
    }
    return new SequentialCommandGroup(
        new FollowPathCommandOurs(drive, limelightSubsystem, "Note Cancel Test"),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }));
  }

  public static Command SourceCenter3Piece(CommandBlocks commandBlocks, DriveSubsystem drive,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, TriggerSubsystem triggerSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem, TriangleInterpolator triangles) {
    if (!SubsystemConstants.useDrive || !SubsystemConstants.useIntake || !SubsystemConstants.useShooter
        || !SubsystemConstants.useLimelight || !SubsystemConstants.useTrigger) {
      return null;
    }
    return new SequentialCommandGroup(
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition, ShooterConstants.preloadSpeedLeft, ShooterConstants.preloadSpeedRight).withTimeout(1.5),
        commandBlocks.instantSetPivot(PivotConstants.podiumPosition),
        new FollowPathCommandOurs(drive, limelightSubsystem, "Source Center 3 Piece Part 1", true),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(1.5),
        commandBlocks.followPathWithPresetShot("Source Center 3 Piece Part 2", false),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5),
        new FollowPathCommandOurs(drive, limelightSubsystem, "Source Center 3 Piece Part 3"),
        new DriveToNoteCommand(drive, limelightSubsystem, intakeSubsystem, triggerSubsystem, () -> {
          return DriveConstants.driveToNoteSpeed;
        }).withTimeout(1.5),
        commandBlocks.followPathWithPresetShot("Source Center 3 Piece Part 3", false),
        new DrivePointedToSpeakerCommand(drive, limelightSubsystem, poseEstimatorSubsystem, triangles).withTimeout(1),
        commandBlocks.fireGamePieceCommand().withTimeout(1.5));
  }

  public static Command DoNothing() {
    return new WaitCommand(10);
  }

  private Autos() {
    throw new UnsupportedOperationException("Now watch me whip! Now watch me nae nae!");
  }
}
