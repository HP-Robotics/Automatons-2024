// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetShooterCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  // private final Joystick m_opJoystick = new
  // Joystick(OperatorConstants.kOperatorControllerPort);
  // The robot's subsystems and commands are defined here...
  final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem() : null;

  private final ShooterSubsystem m_shooterSubsystem = SubsystemConstants.useShooter ? new ShooterSubsystem() : null;
  private final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;

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
                m_robotDrive.drive(
<<<<<<< HEAD
                    Math.signum(m_driveJoystick.getRawAxis(1))
                        * Math.pow(MathUtil.applyDeadband(m_driveJoystick.getRawAxis(1),
                            OperatorConstants.driveJoystickDeadband), 2)
                        * -1
                        * DriveConstants.kMaxSpeed,
                    Math.signum(m_driveJoystick.getRawAxis(0))
                        * Math.pow(MathUtil.applyDeadband(m_driveJoystick.getRawAxis(0),
                            OperatorConstants.driveJoystickDeadband), 2)
                        * -1
                        * DriveConstants.kMaxSpeed,
                    MathUtil.applyDeadband(m_driveJoystick.getRawAxis(2), OperatorConstants.turnJoystickDeadband) * -1
=======
                    // TODO MENTOR:  are deadbands good?  Do we want to try to tweak turning so it's easier to turn a small amount?
                    Math.signum(m_joystick.getRawAxis(1))
                        * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.15), 2) * -1
                        * DriveConstants.kMaxSpeed,
                    Math.signum(m_joystick.getRawAxis(0))
                        * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.15), 2) * -1
                        * DriveConstants.kMaxSpeed,
                    MathUtil.applyDeadband(m_joystick.getRawAxis(4), 0.15) * -1
>>>>>>> origin/Theo
                        * DriveConstants.kMaxAngularSpeed,
                    m_robotDrive.m_fieldRelative);
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
    // new JoystickButton(m_driveJoystick, 2).onTrue(new InstantCommand(m_robotDrive::forceRobotRelative, m_robotDrive));
    // new JoystickButton(m_driveJoystick, 2).onFalse(new
    // InstantCommand(m_robotDrive::forceFieldRelative, m_robotDrive));
    m_driveJoystick.button(6).whileTrue(new InstantCommand(m_robotDrive::resetYaw));
    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // // cancelling on release.
     }
    if(SubsystemConstants.useShooter){
      m_driveJoystick.button(2).whileTrue(new SetShooterCommand(m_shooterSubsystem));
    }
    if(SubsystemConstants.useIntake){
      m_driveJoystick.axisGreaterThan(3, 0.1).whileTrue(new IntakeCommand(m_intakeSubsystem));
    }

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void resetDriveOffsets() {
    if (SubsystemConstants.useDrive) {
      m_robotDrive.resetOffsets();
    }

  }

}
