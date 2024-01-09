// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveParkingBrakeCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Joystick m_joystick = new Joystick(1);
  private final Joystick m_opJoystick = new Joystick(0);
  // The robot's subsystems and commands are defined here...
    final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem() : null;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.

          new RunCommand(

              () -> {
                m_robotDrive.drive(
                    // TODO MENTOR:  are deadbands good?  Do we want to try to tweak turning so it's easier to turn a small amount?
                    Math.signum(m_joystick.getRawAxis(1))
                        * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.05), 2) * -1
                        * DriveConstants.kMaxSpeed,
                    Math.signum(m_joystick.getRawAxis(0))
                        * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.05), 2) * -1
                        * DriveConstants.kMaxSpeed,
                    MathUtil.applyDeadband(m_joystick.getRawAxis(2), 0.1) * -1
                        * DriveConstants.kMaxAngularSpeed,
                    //0.2 * DriveConstants.kMaxSpeed, 0, 0,
                    m_robotDrive.m_fieldRelative);
              },
              m_robotDrive));
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
 new JoystickButton(m_joystick, 2).onTrue(new InstantCommand(m_robotDrive::forceRobotRelative, m_robotDrive));
      new JoystickButton(m_joystick, 2).onFalse(new InstantCommand(m_robotDrive::forceFieldRelative, m_robotDrive));
      //new JoystickButton(m_joystick, 10).onTrue(new InstantCommand(m_robotDrive::resetYaw, m_robotDrive)); No go north for now
      new JoystickButton(m_joystick, 16).whileTrue(new DriveParkingBrakeCommand(m_robotDrive));
      new JoystickButton(m_joystick, 5).whileTrue(new RunCommand(
          () -> {
            m_robotDrive.drive(
                // TODO MENTOR:  are deadbands good?  Do we want to try to tweak turning so it's easier to turn a small amount? Also this is slowmode
                Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.1), 1) * -1 * DriveConstants.kSlowSpeed,
                Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.1), 1) * -1
                    * DriveConstants.kSlowSpeed,
                MathUtil.applyDeadband(m_joystick.getRawAxis(2), 0.2) * -1
                    * DriveConstants.kSlowAngularspeed,
                //0.2 * DriveConstants.kMaxSpeed, 0, 0,
                m_robotDrive.m_fieldRelative);
          },
          m_robotDrive));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
    public void resetDriveOffsets() {

    if (SubsystemConstants.useDrive) {
      m_robotDrive.resetOffsets();

    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

  }

 
}
