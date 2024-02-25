// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.DrivePointedToSpeakerCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CommandBlocks;
import frc.robot.commands.FollowPathCommandOurs;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeStatesCommand;
import frc.robot.commands.PivotMagicCommand;
import frc.robot.commands.PivotManualCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.TriggerCommand;
import frc.robot.commands.TriggerStatesCommand;
import frc.robot.commands.Autos;
import frc.robot.BeamBreak;

import javax.sound.sampled.CompoundControl;

import com.ctre.phoenix6.controls.NeutralOut;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SnuffilatorSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private CommandBlocks compoundCommands; // TODO: Pick a better name

  // The robot's subsystems and commands are defined here...
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem();
  final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem(m_PoseEstimatorSubsystem) : null;
  private final LimelightSubsystem m_limelightSubsystem = SubsystemConstants.useLimelight
      ? new LimelightSubsystem(m_PoseEstimatorSubsystem)
      : null;

  private final ShooterSubsystem m_shooterSubsystem = SubsystemConstants.useShooter ? new ShooterSubsystem(m_opJoystick) : null;
  private final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  private final PivotSubsystem m_pivotSubsystem = SubsystemConstants.usePivot ? new PivotSubsystem() : null;
  private final ClimbSubsystem m_climberSubsystem = SubsystemConstants.useClimber ? new ClimbSubsystem() : null;
  private final TriggerSubsystem m_triggerSubsystem = SubsystemConstants.useShooter ? new TriggerSubsystem() : null;
  private final SnuffilatorSubsystem m_snuffilatorSubsystem = SubsystemConstants.useSnuffilator
      ? new SnuffilatorSubsystem()
      : null;

  private final PowerDistribution pdh = new PowerDistribution(1,ModuleType.kRev);

  private final SendableChooser<String> m_chooseAutos;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    pdh.setSwitchableChannel(true);

    if (SubsystemConstants.useDataManager) {
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

    if (SubsystemConstants.useIntake && SubsystemConstants.useTrigger) {
      m_intakeSubsystem.setDefaultCommand(
          new IntakeStatesCommand(m_intakeSubsystem, m_triggerSubsystem.m_beamBreak));
      m_triggerSubsystem.setDefaultCommand(
          new TriggerStatesCommand(m_triggerSubsystem, m_triggerSubsystem.m_beamBreak));
    }

    m_chooseAutos = new SendableChooser<String>();
    m_chooseAutos.addOption("Center Down", "CenterDown");
    m_chooseAutos.addOption("Four Piece", "FourPiece");
    m_chooseAutos.addOption("Grand Theft Auto", "GrandTheftAuto");
    m_chooseAutos.addOption("Basic Amp", "BasicAmp");
    m_chooseAutos.addOption("Intermediate Amp", "IntermediateAmp");
    m_chooseAutos.addOption("Four Piece Center", "FourPieceCenter");
    m_chooseAutos.addOption("Three Piece Center", "Three Piece Center");
    m_chooseAutos.addOption("Test Path 5", "TestPath5");
    m_chooseAutos.addOption("Shoot Preload Far Away","ShootPreloadFarAway");
    m_chooseAutos.addOption("Only Shoot", "OnlyShoot");
    m_chooseAutos.setDefaultOption("Do Nothing", "DoNothing");

    SmartDashboard.putData("Auto Chooser", m_chooseAutos);

    compoundCommands = new CommandBlocks(m_robotDrive, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
        m_pivotSubsystem, m_snuffilatorSubsystem);
    configureCommands();
    configureBindings();

  }

  private void configureCommands() {
    NamedCommands.registerCommand("startIntaking", compoundCommands.startIntaking());
    NamedCommands.registerCommand("stopIntaking", compoundCommands.stopIntaking());
    NamedCommands.registerCommand("runShooter", new SetShooterCommand(m_shooterSubsystem, null, null));
    NamedCommands.registerCommand("stopShooter", new SetShooterCommand(m_shooterSubsystem, 0.0, 0.0));
  }

  private void configureBindings() {

    if (SubsystemConstants.useDrive) {
      m_driveJoystick.button(OperatorConstants.resetYawButton).whileTrue(new InstantCommand(m_robotDrive::resetYaw));
      Trigger fieldRelativeTrigger = OperatorConstants.useXbox
          ? new Trigger(m_driveJoystick.axisGreaterThan(2, 0.1))
          : new Trigger(m_driveJoystick.button(OperatorConstants.fieldRelativeButton));
      // fieldRelativeTrigger.onTrue(new InstantCommand(() ->
      // m_robotDrive.setFieldRelative(false)));
      // fieldRelativeTrigger.onFalse(new InstantCommand(() ->
      // m_robotDrive.setFieldRelative(true)));
      // m_driveJoystick.button(7).whileTrue(new FollowPathCommand(m_robotDrive, "Test
      // Path"));
      // m_driveJoystick.button(8).whileTrue(new FollowPathCommand(m_robotDrive, "Test
      // Path Line"));
      // m_driveJoystick.button(4).whileTrue(new RunCommand(()->
      // m_robotDrive.drivePointedTowardsAngle(m_driveJoystick, new Rotation2d(0))));
    }

    if (SubsystemConstants.useShooter) {
      m_opJoystick.axisGreaterThan(3, 0.1).whileTrue(
          new SetShooterCommand(m_shooterSubsystem, null, null));
      // TODO add trigger if statement
      m_opJoystick.button(3).whileTrue(compoundCommands.fireButtonHold());
      m_opJoystick.button(6).whileTrue(
          new SetShooterCommand(m_shooterSubsystem, ShooterConstants.shooterSpeedAmp, ShooterConstants.shooterSpeedAmp)
              .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
      m_driveJoystick.button(OperatorConstants.yuckButton).whileTrue(compoundCommands.yuckButtonHold());
    }

    if (SubsystemConstants.useClimber) {
      m_driveJoystick.button(OperatorConstants.climberButton).whileTrue(new ClimberCommand(m_climberSubsystem));
    }

    if (SubsystemConstants.useIntake) {
      Trigger intakeTrigger = OperatorConstants.useXbox
          ? new Trigger(m_driveJoystick.axisGreaterThan(3, 0.1))
          : new Trigger(m_driveJoystick.button(OperatorConstants.intakeButton));
      intakeTrigger.whileTrue(compoundCommands.intakeButtonHold());
    }

    if (SubsystemConstants.usePivot) {
      m_opJoystick.povRight().whileTrue(new PivotManualCommand(m_pivotSubsystem, PivotConstants.manualSpeed));
      m_opJoystick.povLeft().whileTrue(new PivotManualCommand(m_pivotSubsystem, -PivotConstants.manualSpeed));
      m_opJoystick.button(7).onTrue(new InstantCommand(m_pivotSubsystem::togglePID));
      m_opJoystick.button(1)
          .onTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.subwooferPosition)));
      m_opJoystick.button(2)
          .whileTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.ampPosition)));
      m_opJoystick.button(4)
          .whileTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.podiumPosition)));
    }
    if (SubsystemConstants.useDrive && SubsystemConstants.useLimelight) {
      m_driveJoystick.button(OperatorConstants.drivePointedToSpeakerButton)
          .whileTrue(new DrivePointedToSpeakerCommand(m_robotDrive, m_limelightSubsystem, m_driveJoystick)); 
      m_opJoystick.axisGreaterThan(2, 0.1).whileTrue(new PivotMagicCommand(m_pivotSubsystem, m_limelightSubsystem));    }

    if (SubsystemConstants.useSnuffilator) {
      new Trigger(() -> {return m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition;})
          .onTrue(compoundCommands.moveSnuffilator(true))
          .onFalse(compoundCommands.moveSnuffilator(false));
    }
  }

  // pulls beambreak every millisecond
  public void fastBeamBreakCheckIntake() {
    if (!SubsystemConstants.useIntake) {
      return;
    }
    if (m_intakeSubsystem.beambreakState
        || (!m_intakeSubsystem.intakeFire && !m_intakeSubsystem.intakeOn && !m_intakeSubsystem.intakeYuck)) {
      return;
    }
    if (!m_triggerSubsystem.m_beamBreak.beamBroken()) {
      return;
    }
    m_intakeSubsystem.beambreakState = true;
    if (m_intakeSubsystem.intakeYuck || m_intakeSubsystem.intakeFire) {
      return;
    }
    m_intakeSubsystem.m_motor.setControl(new NeutralOut());
  }

  public void fastBeamBreakCheckTrigger() {
    if (!SubsystemConstants.useTrigger) {
      return;
    }
    if (m_triggerSubsystem.beambreakState
        || (!m_triggerSubsystem.triggerFire && !m_triggerSubsystem.triggerOn && !m_triggerSubsystem.triggerYuck)) {
      return;
    }
    if (!m_triggerSubsystem.m_beamBreak.beamBroken()) {
      return;
    }
    m_triggerSubsystem.beambreakState = true;
    if (m_triggerSubsystem.triggerYuck || m_triggerSubsystem.triggerFire) {
      return;
    }
    // NeutralOut neutral = new NeutralOut();
    // neutral.UpdateFreqHz = 1000;
    m_triggerSubsystem.m_triggerMotor.setControl(new NeutralOut());
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

  public Command getAutonomousCommand() { //TODO put by auto chooser
    if (m_chooseAutos.getSelected() == "CenterDown") {
      return Autos.CenterDown(compoundCommands, m_robotDrive, m_shooterSubsystem);
    }
    if (m_chooseAutos.getSelected() == "FourPiece") {
      return Autos.FourPiece(compoundCommands, m_robotDrive, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
          m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "FourPieceCenter") {
      return Autos.FourPieceCenter(compoundCommands, m_robotDrive, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
          m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "GrandTheftAuto") {
      return Autos.GrandTheftAuto(m_robotDrive);
    }
    if (m_chooseAutos.getSelected() == "BasicAmp") {
      return Autos.BasicAmp(compoundCommands, m_robotDrive, m_intakeSubsystem, m_shooterSubsystem);
    }
    if (m_chooseAutos.getSelected() == "IntermediateAmp") {
      return Autos.IntermediateAmp(compoundCommands, m_robotDrive, m_intakeSubsystem, m_shooterSubsystem);
    }
    if (m_chooseAutos.getSelected() == "TestPath5") {
      return Autos.FiveMeterTest(m_robotDrive);
    }
    if (m_chooseAutos.getSelected() == "ShootPreloadFarAway") {
      return Autos.ShootPreloadFarAway(compoundCommands, m_robotDrive, m_shooterSubsystem, m_limelightSubsystem, m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "OnlyShoot") {
      return Autos.OnlyShoot(compoundCommands, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem, m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "DoNothing") {
      return Autos.DoNothing();
    } else {
      return Autos.DoNothing();
    }
  }
}
