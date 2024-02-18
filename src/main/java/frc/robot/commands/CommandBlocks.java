package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;

public class CommandBlocks {
    IntakeSubsystem m_intakeSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    TriggerSubsystem m_triggerSubsystem;
    PivotSubsystem m_pivotSubsystem;
  
  public CommandBlocks(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
    ShooterSubsystem shooterSubsystem, TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_triggerSubsystem = triggerSubsystem;
  }

  public Command fireGamePieceCommand() {
        return new ParallelDeadlineGroup(
            new WaitCommand(1).until(() -> {return m_shooterSubsystem.atSpeed() && m_pivotSubsystem.atPosition();})
          .andThen(new TriggerCommand(m_triggerSubsystem, true, m_intakeSubsystem).withTimeout(0.3)),
          new SetShooterCommand(m_shooterSubsystem, null, null),
          new InstantCommand(() -> m_pivotSubsystem.setPosition(0.43)),
          new InstantCommand(() -> {System.out.println("firing game piece");})
        );
  }

  public Command intakeButtonHold() {
    return new ParallelCommandGroup(
      new StartEndCommand(m_triggerSubsystem::intakeButtonPressed, m_triggerSubsystem::intakeButtonReleased), 
      new StartEndCommand(m_intakeSubsystem::intakeButtonPressed, m_intakeSubsystem::intakeButtonReleased)
    );
  }
  
  public Command fireButtonHold() {
    return new ParallelCommandGroup(
      new StartEndCommand(m_triggerSubsystem::fireButtonPressed, m_triggerSubsystem::fireButtonReleased), 
      new StartEndCommand(m_intakeSubsystem::fireButtonPressed, m_intakeSubsystem::fireButtonReleased)
    );
  }
}
