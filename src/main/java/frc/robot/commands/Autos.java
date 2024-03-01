// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }

  public static Command FourPiece(CommandBlocks commandBlocks, DriveSubsystem drive, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "4 Piece part 1"),
        commandBlocks.fireGamePieceCommand(PivotConstants.note1_3Position),
        new FollowPathCommandOurs(drive, "4 Piece part 2"),
        commandBlocks.fireGamePieceCommand(PivotConstants.note2Position),
        new FollowPathCommandOurs(drive, "4 Piece Part 3"),
        commandBlocks.fireGamePieceCommand(PivotConstants.note1_3Position));
  }

  public static Command FourPieceCenter(CommandBlocks commandBlocks, DriveSubsystem drive,
      IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
      TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem, LimelightSubsystem limelightSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
        new InstantCommand(() -> pivotSubsystem.setPosition(PivotConstants.subwooferPosition)),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new InstantCommand(() -> pivotSubsystem.setPosition(pivotSubsystem.getMagicAngle(
            limelightSubsystem.getDistanceTo(new Pose2d(2.65, 6.83, Rotation2d.fromDegrees(35.33)),
                LimelightConstants.aprilTag7)))),
        new FollowPathCommandOurs(drive, "4 Piece Center Part 1"),
        commandBlocks.fireGamePieceCommand(pivotSubsystem.getMagicAngle(
            limelightSubsystem.getDistanceTo(new Pose2d(2.65, 6.83, Rotation2d.fromDegrees(35.33)),
                LimelightConstants.aprilTag7))),
        new InstantCommand(() -> pivotSubsystem.setPosition(PivotConstants.note2Position)),
        new FollowPathCommandOurs(drive, "4 Piece Center Part 2"),
        commandBlocks.fireGamePieceCommand(PivotConstants.note2Position),
        new InstantCommand(() -> pivotSubsystem.setPosition(pivotSubsystem.getMagicAngle(
            limelightSubsystem.getDistanceTo(new Pose2d(2.9, 4.25, Rotation2d.fromDegrees(-26.33)),
                LimelightConstants.aprilTag7)))),
        new FollowPathCommandOurs(drive, "4 Piece Center Part 3"),
        commandBlocks.fireGamePieceCommand(pivotSubsystem.getMagicAngle(
            limelightSubsystem.getDistanceTo(new Pose2d(2.9, 4.25, Rotation2d.fromDegrees(-26.33)),
                LimelightConstants.aprilTag7))),
        new InstantCommand(shooterSubsystem::stopShooter));
  }

  public static Command CenterDown(CommandBlocks commandBlocks, DriveSubsystem drive,
      ShooterSubsystem shooterSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
        new FollowPathCommandOurs(drive, "Center Down Part 1"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "Center Down Part 2"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "Center Down Part 3"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition));
  }

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
        new FollowPathCommandOurs(drive, "Center Down Part 1"),
        commandBlocks.fireGamePieceCommand(pivotSubsystem.getMagicAngle(
            limelightSubsystem.getDistanceTo(new Pose2d(3.38, 3.1, Rotation2d.fromDegrees(-35.94)),
                LimelightConstants.aprilTag7))));
    // TODO: Load the last point from the path
  }

  public static Command BasicAmp(CommandBlocks commandBlocks, DriveSubsystem drive, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem) {
    if (!SubsystemConstants.useDrive || !SubsystemConstants.useIntake || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
        new FollowPathCommandOurs(drive, "Basic Amp Part 1"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "Basic Amp Part 2"),
        new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.additionalIntakeTime));
  }

  public static Command GrandTheftAuto(DriveSubsystem drive) {
    if (!SubsystemConstants.useDrive) {
      return null;
    }
    return new SequentialCommandGroup(
        new FollowPathCommandOurs(drive, "Grand Theft Auto Part 1"));
  }

  public static Command IntermediateAmp(CommandBlocks commandBlocks, DriveSubsystem drive,
      IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
        new FollowPathCommandOurs(drive, "Intermediate Amp Part 1"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "Intermediate Amp Part 2"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "Intermediate Amp Part 3"),
        new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.additionalIntakeTime));
  }

  public static Command ThreePieceCenter(CommandBlocks commandBlocks, DriveSubsystem drive,
      IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
      TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "3 Piece Center Part 1"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition),
        new FollowPathCommandOurs(drive, "3 Piece Center Part 2"),
        commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition));
  }

  public static Command FiveMeterTest(DriveSubsystem drive) {
    if (!SubsystemConstants.useDrive) {
      return null;
    } else {
      return new FollowPathCommandOurs(drive, "Test Path 5 Meters");
    }
  }

  public static Command OnlyShoot(CommandBlocks commandBlocks, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useShooter) {
      return new WaitCommand(0);
    }
    return commandBlocks.fireGamePieceCommand(PivotConstants.subwooferPosition);
  }

  public static Command DoNothing() {
    return new WaitCommand(10);
  }

  private Autos() {
    throw new UnsupportedOperationException("Now watch me whip! Now watch me nae nae!");
  }
}
