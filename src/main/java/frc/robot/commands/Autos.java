// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;


public final class Autos {
  /** Example static factory for an autonomous command. */
//   public static Command exampleAuto(ExampleSubsystem subsystem) {
//    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
//  }

  public static Command FourPiece(DriveSubsystem drive) { //TODO add if subsystem statements
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "4 Piece part 1"),
      new WaitCommand(1),
      new FollowPathCommandOurs(drive, "4 Piece part 2"),
      new WaitCommand(1),
      new FollowPathCommandOurs(drive, "4 Piece Part 3")
      );
  }

  public static Command CenterDown(DriveSubsystem drive) {
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Center Down Part 1"),
      new WaitCommand(1),
      new FollowPathCommandOurs(drive, "Center Down Part 2"),
      new WaitCommand(1),
      new FollowPathCommandOurs(drive, "Center Down Part 3")
      );
  }

  public static Command BasicAmp(DriveSubsystem drive) {
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Basic Amp Part 1"),
      new WaitCommand(1),
      new FollowPathCommandOurs(drive, "Basic Amp Part 2"),
      new WaitCommand(1)
      );
    }

  public static Command GrandTheftAuto(DriveSubsystem drive) {
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Grand Theft Auto Part 1")
    );
  }

  public static Command IntermediateAmp(DriveSubsystem drive) {
    return new SequentialCommandGroup(
      new FollowPathCommandOurs(drive, "Intermediate Amp Part 1"),
      new WaitCommand(1),
      new FollowPathCommandOurs(drive, "Intermediate Amp Part 2"),
      new WaitCommand(1),
      new FollowPathCommandOurs(drive, "Intermediate Amp Part 3")
    );
  }

  


  private Autos() {
    throw new UnsupportedOperationException("Now watch me whip! Now watch me nae nae!");
  }


}
