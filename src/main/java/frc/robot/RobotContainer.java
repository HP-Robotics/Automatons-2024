// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PivotManualCommand;
import frc.robot.commands.SetShooterCommand;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
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
    m_chooseAutos.addOption("Yuck", "yuck");
    m_chooseAutos.setDefaultOption("Intermediate Amp", "IntermediateAmp");
    SmartDashboard.putData("Auto Chooser", m_chooseAutos);

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
     // m_driveJoystick.button(7).whileTrue(new FollowPathCommand(m_robotDrive, "Test Path"));
      m_driveJoystick.button(7).whileTrue(new FollowPathCommand(m_robotDrive, "Test Path Spinny Line"));
      m_driveJoystick.button(4).whileTrue(new RunCommand(()-> m_robotDrive.drivePointedTowardsAngle(m_driveJoystick, new Rotation2d(0))));
      
    }

    if(SubsystemConstants.useShooter){
      m_opJoystick.button(2).whileTrue(new SetShooterCommand(m_shooterSubsystem));
    }

    if (SubsystemConstants.useIntake) {
      m_driveJoystick.axisGreaterThan(3, 0.1).whileTrue(new IntakeCommand(m_intakeSubsystem));
    }
    if(SubsystemConstants.usePivot){
      m_opJoystick.povRight().whileTrue(new PivotManualCommand(m_pivotSubsystem, PivotConstants.manualSpeed));
      m_opJoystick.povLeft().whileTrue(new PivotManualCommand(m_pivotSubsystem, -PivotConstants.manualSpeed));
      m_opJoystick.button(7).onTrue(new InstantCommand(m_pivotSubsystem::togglePID));
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
