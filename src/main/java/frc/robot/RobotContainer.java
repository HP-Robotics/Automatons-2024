// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PivotManualCommand;
import frc.robot.commands.PivotSetPositionCommand;
import frc.robot.commands.SetShooterCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final CommandJoystick m_operatorJoystick = new CommandJoystick(OperatorConstants.kOperatorControllerPort);
  // private final Joystick m_opJoystick = new
  // Joystick(OperatorConstants.kOperatorControllerPort);
  // The robot's subsystems and commands are defined here...
  final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem() : null;
  private final LimelightSubsystem m_limelightSubsystem = SubsystemConstants.useLimelight ? new LimelightSubsystem()
      : null;

  private final ShooterSubsystem m_shooterSubsystem = SubsystemConstants.useShooter ? new ShooterSubsystem() : null;
  private final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  private final PivotSubsystem m_pivotSubsystem = SubsystemConstants.usePivot ? new PivotSubsystem() : null;

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
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    if (SubsystemConstants.useDrive) {
      // new JoystickButton(m_driveJoystick, 2).onTrue(new
      // InstantCommand(m_robotDrive::forceRobotRelative, m_robotDrive));
      // new JoystickButton(m_driveJoystick, 2).onFalse(new
      // InstantCommand(m_robotDrive::forceFieldRelative, m_robotDrive));
      m_driveJoystick.button(8).whileTrue(new InstantCommand(m_robotDrive::resetYaw));
      m_driveJoystick.button(7).whileTrue(new FollowPathCommand(m_robotDrive, "Test Path"));
      m_driveJoystick.button(8).whileTrue(new FollowPathCommand(m_robotDrive, "Test Path Line"));
      m_driveJoystick.button(4).whileTrue(new RunCommand(()-> m_robotDrive.drivePointedTowardsAngle(m_driveJoystick, new Rotation2d(0))));
      
    }

    if(SubsystemConstants.useShooter){
      m_operatorJoystick.axisGreaterThan(3, 0.1).whileTrue(new SetShooterCommand(m_shooterSubsystem)); //Toggle shooter wheels
    }

    if (SubsystemConstants.useIntake) {
      m_driveJoystick.axisGreaterThan(3, 0.1).whileTrue(new IntakeCommand(m_intakeSubsystem));
    }
    if(SubsystemConstants.usePivot){
      m_operatorJoystick.povRight().whileTrue(new PivotManualCommand(m_pivotSubsystem, PivotConstants.manualSpeed));
      m_operatorJoystick.povLeft().whileTrue(new PivotManualCommand(m_pivotSubsystem, -PivotConstants.manualSpeed));
      m_operatorJoystick.button(2).whileTrue(new PivotSetPositionCommand(m_pivotSubsystem, PivotConstants.positionStage));
      m_operatorJoystick.button(1).whileTrue(new PivotSetPositionCommand(m_pivotSubsystem, PivotConstants.positionAmp));
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
    return Autos.FourPiece(m_robotDrive);
    //return null;
  }
}
