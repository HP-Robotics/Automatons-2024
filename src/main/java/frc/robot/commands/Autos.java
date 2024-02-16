// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;


public final class Autos {
  /** Example static factory for an autonomous command. */
//   public static Command exampleAuto(ExampleSubsystem subsystem) {
//    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
//  }

  public static Command FourPiece(CommandBlocks commandBlocks, DriveSubsystem drive, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, 
  TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem) { 
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
      commandBlocks.fireGamePieceCommand(),
      new ParallelCommandGroup(
        new WaitCommand(1.0),
        new FollowPathCommandOurs(drive, "4 Piece part 1")
      ),
     
      
      new ParallelCommandGroup(
        new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.additionalIntakeTime),
        new TriggerCommand(triggerSubsystem, false, intakeSubsystem)
        .andThen(commandBlocks.fireGamePieceCommand())
      ),
      new ParallelCommandGroup(
        new FollowPathCommandOurs(drive, "4 Piece part 2"),
        new WaitCommand(1.0)
      ),
      new ParallelCommandGroup(
        new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.additionalIntakeTime),
        commandBlocks.fireGamePieceCommand()
      ),
      new ParallelCommandGroup(
        new FollowPathCommandOurs(drive, "4 Piece Part 3"),
        new WaitCommand(1.0)
      ),
      
      new ParallelCommandGroup(
        new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.additionalIntakeTime),
        commandBlocks.fireGamePieceCommand()
      )
      );
  }

  public static Command CenterDown(CommandBlocks commandBlocks, DriveSubsystem drive, ShooterSubsystem shooterSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Center Down Part 1"),
      commandBlocks.fireGamePieceCommand(),
      new FollowPathCommandOurs(drive, "Center Down Part 2"),
      commandBlocks.fireGamePieceCommand(),
      new FollowPathCommandOurs(drive, "Center Down Part 3"),
      commandBlocks.fireGamePieceCommand()
      );
  }

  public static Command BasicAmp(CommandBlocks commandBlocks, DriveSubsystem drive, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shooterSubsystem) {
    if (!SubsystemConstants.useDrive || !SubsystemConstants.useIntake || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Basic Amp Part 1"),
      commandBlocks.fireGamePieceCommand(),
      new FollowPathCommandOurs(drive, "Basic Amp Part 2"),
      new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.additionalIntakeTime)
      );
    }

  public static Command GrandTheftAuto(DriveSubsystem drive) {
    if (!SubsystemConstants.useDrive) {
      return null;
    }
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Grand Theft Auto Part 1")
    );
  }

  public static Command IntermediateAmp(CommandBlocks commandBlocks, DriveSubsystem drive, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    if (!SubsystemConstants.useIntake || !SubsystemConstants.useDrive || !SubsystemConstants.useShooter) {
      return null;
    }
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Intermediate Amp Part 1"),
      commandBlocks.fireGamePieceCommand(),
      new FollowPathCommandOurs(drive, "Intermediate Amp Part 2"),
      commandBlocks.fireGamePieceCommand(),
      new FollowPathCommandOurs(drive, "Intermediate Amp Part 3"),
      new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.additionalIntakeTime)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("Now watch me whip! Now watch me nae nae!");
  }
}
