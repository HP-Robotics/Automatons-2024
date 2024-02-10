// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.DrivePointedToSpeakerCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.FollowPathCommandOurs;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PivotManualCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.TriggerCommand;
import frc.robot.commands.Autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandJoystick m_driveJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick m_opJoystick = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  // The robot's subsystems and commands are defined here...
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem = 
    new PoseEstimatorSubsystem();
  final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem(m_PoseEstimatorSubsystem) : null;
  private final LimelightSubsystem m_limelightSubsystem = SubsystemConstants.useLimelight ? 
      new LimelightSubsystem(m_PoseEstimatorSubsystem) : null;

  private final ShooterSubsystem m_shooterSubsystem = SubsystemConstants.useShooter ? new ShooterSubsystem() : null;
  private final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  private final PivotSubsystem m_pivotSubsystem = SubsystemConstants.usePivot ? new PivotSubsystem() : null;
  private final ClimbSubsystem m_climberSubsystem = SubsystemConstants.useClimber ? new ClimbSubsystem() : null;
  private final TriggerSubsystem m_triggerSubsystem = SubsystemConstants.useShooter ? new TriggerSubsystem() : null;

  private final SendableChooser<String> m_chooseAutos;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (SubsystemConstants.useDataManger) {
      DataLogManager.start();
    }

    if (SubsystemConstants.useDrive) {
      m_robotDrive.setDefaultCommand(

          new RunCommand(
              () -> {
                m_robotDrive.driveWithJoystick(m_driveJoystick);
              },
              m_robotDrive));
    }

    NamedCommands.registerCommand("runIntake", m_intakeSubsystem.runOnce(() -> m_intakeSubsystem.runIntake(IntakeConstants.intakeSpeed)));
    NamedCommands.registerCommand("stopIntake", m_intakeSubsystem.runOnce(() -> m_intakeSubsystem.runIntake(0)));

    m_chooseAutos = new SendableChooser<String>();
    m_chooseAutos.addOption("Center Down", "CenterDown");
    m_chooseAutos.addOption("Four Piece", "FourPiece");
    m_chooseAutos.addOption("Grand Theft Auto", "GrandTheftAuto");
    m_chooseAutos.addOption("Basic Amp", "BasicAmp");
    m_chooseAutos.setDefaultOption("Intermediate Amp", "IntermediateAmp");
    SmartDashboard.putData("Auto Chooser", m_chooseAutos);

    configureBindings();
  }

  private void configureBindings() {

    if (SubsystemConstants.useDrive) {
      m_driveJoystick.button(8).whileTrue(new InstantCommand(m_robotDrive::resetYaw));
      m_driveJoystick.button(7).whileTrue(new FollowPathCommandOurs(m_robotDrive, "Test Path"));
      m_driveJoystick.button(9).whileTrue(new FollowPathCommandOurs(m_robotDrive, "Test Path Line"));
      m_driveJoystick.button(4)
          .whileTrue(new RunCommand(() -> m_robotDrive.drivePointedTowardsAngle(m_driveJoystick, new Rotation2d(0))));
    }

    if (SubsystemConstants.useShooter) {
      m_opJoystick.axisGreaterThan(3, 0.1).whileTrue(
        new SetShooterCommand(m_shooterSubsystem));
      m_opJoystick.button(3).whileTrue(new ParallelCommandGroup(
        new SetShooterCommand(m_shooterSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
        new WaitUntilCommand(m_shooterSubsystem::atSpeed).andThen(new TriggerCommand(m_triggerSubsystem, true)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
      ));
    }

    

    if (SubsystemConstants.useClimber) {
      m_driveJoystick.button(10).whileTrue(new ClimberCommand(m_climberSubsystem));
    }

    if (SubsystemConstants.useIntake) {
      m_driveJoystick.axisGreaterThan(3, 0.1).whileTrue(new ParallelCommandGroup(
        new IntakeCommand(m_intakeSubsystem),
        new TriggerCommand(m_triggerSubsystem, false).asProxy() // TODO: Restart if cancelled
        ));
    }
    
    if(SubsystemConstants.usePivot){
      m_opJoystick.povRight().whileTrue(new PivotManualCommand(m_pivotSubsystem, PivotConstants.manualSpeed));
      m_opJoystick.povLeft().whileTrue(new PivotManualCommand(m_pivotSubsystem, -PivotConstants.manualSpeed));
      m_opJoystick.button(7).onTrue(new InstantCommand(m_pivotSubsystem::togglePID));
    }
    if (SubsystemConstants.useDrive && SubsystemConstants.useLimelight){
      m_driveJoystick.button(5).whileTrue(new DrivePointedToSpeakerCommand(m_robotDrive, m_limelightSubsystem, m_driveJoystick));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void resetDriveOffsets() {
    if (SubsystemConstants.useDrive) {
      m_robotDrive.resetOffsets();
    }
  }

  public Command getAutonomousCommand() {
    if (m_chooseAutos.getSelected() == "CenterDown") {
      return Autos.CenterDown(m_robotDrive);
    } 
    if (m_chooseAutos.getSelected() == "FourPiece") {
      return Autos.FourPiece(m_robotDrive);
    }
    if (m_chooseAutos.getSelected() == "GrandTheftAuto") {
      return Autos.GrandTheftAuto(m_robotDrive);
    } 
    if (m_chooseAutos.getSelected() == "BasicAmp") {
      return Autos.BasicAmp(m_robotDrive);
    }
    if (m_chooseAutos.getSelected() == "IntermediateAmp") {
      return Autos.IntermediateAmp(m_robotDrive);
    }
    else {
      return null;
    }
  }
}
