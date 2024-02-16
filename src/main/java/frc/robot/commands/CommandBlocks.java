package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
    public Command fireGamePieceCommand() {
        return new ParallelDeadlineGroup(
            new WaitCommand(1).until(m_shooterSubsystem::atSpeed).until(m_pivotSubsystem::atPosition)
          .andThen(new TriggerCommand(m_triggerSubsystem, true, m_intakeSubsystem).withTimeout(0.3)),
          new SetShooterCommand(m_shooterSubsystem),
          new InstantCommand(() -> m_pivotSubsystem.setPosition(0.43)),
          new InstantCommand(() -> {System.out.println("firing game piece");})
        );
    }
    public CommandBlocks(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
        ShooterSubsystem shooterSubsystem, TriggerSubsystem triggerSubsystem, PivotSubsystem pivotSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_pivotSubsystem = pivotSubsystem;
        m_triggerSubsystem = triggerSubsystem;
    }
}