// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.DrivePointedToNoteCommand;
import frc.robot.commands.DrivePointedToSpeakerCommand;
import frc.robot.commands.DriveToNoteCommand;
import frc.robot.commands.CommandBlocks;
import frc.robot.commands.IntakeStatesCommand;
import frc.robot.commands.OperatorRumbleCommand;
import frc.robot.commands.PivotMagicCommand;
import frc.robot.commands.PivotManualCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.TriggerStatesCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.controls.NeutralOut;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final CommandJoystick m_driveJoystick = new CommandJoystick(ControllerConstants.kDriverControllerPort);
  private final CommandJoystick m_opJoystick = new CommandJoystick(ControllerConstants.kOperatorControllerPort);
  private CommandBlocks compoundCommands; // TODO: Pick a better name
  public final Field2d m_autoPose = new Field2d();
  public List<Pose2d> m_autoPath = new ArrayList<>();

  // The robot's subsystems and commands are defined here...
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem();
  final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem(m_PoseEstimatorSubsystem) : null; // TODO:
                                                                                                                         // m_driveSubsystem
  private final LimelightSubsystem m_limelightSubsystem = SubsystemConstants.useLimelight
      ? new LimelightSubsystem(m_PoseEstimatorSubsystem)
      : null;

  private final ShooterSubsystem m_shooterSubsystem = SubsystemConstants.useShooter ? new ShooterSubsystem()
      : null;
  private final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  private final PivotSubsystem m_pivotSubsystem = SubsystemConstants.usePivot ? new PivotSubsystem() : null;
  private final ClimbSubsystem m_climberSubsystem = SubsystemConstants.useClimber ? new ClimbSubsystem() : null;
  private final TriggerSubsystem m_triggerSubsystem = SubsystemConstants.useShooter ? new TriggerSubsystem() : null;
  private final SnuffilatorSubsystem m_snuffilatorSubsystem = SubsystemConstants.useSnuffilator
      ? new SnuffilatorSubsystem()
      : null;
  private final TriangleInterpolator m_magicInterpolator = new TriangleInterpolator(4);
  private final PowerDistribution pdh = new PowerDistribution();

  private final SendableChooser<String> m_chooseAutos;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    TriangleInterpolator.addDuluthMagic(m_magicInterpolator);
    m_magicInterpolator.addCalibratedPoint(2.0, 7.78, 0, 0, 0.374, 0);
    double startTime = Timer.getFPGATimestamp();
    m_magicInterpolator.makeTriangles();
    double triangleTime = Timer.getFPGATimestamp();
    // m_magicInterpolator.draw("/home/lvuser/shooterSpeedLeftTestImage.png", 500, 500, 0, 8.27, 8.27, 0, 0, 40, 60);
    // m_magicInterpolator.draw("/home/lvuser/shooterSpeedRightTestImage.png", 500, 500, 0, 8.27, 8.27, 0, 1, 40, 60);
    m_magicInterpolator.draw("/home/lvuser/pivotAngleTestImage.png", 100, 100, 0, 8.27, 8.27, 0, 2, 0.3, 0.5);
    // m_magicInterpolator.draw("/home/lvuser/headingTestImage.png", 500, 500, 0, 8.27, 8.27, 0, 3, 0, 2 * Math.PI);
    double TenKTesTime = Timer.getFPGATimestamp();
    System.out.println(triangleTime-startTime);
    System.out.println(TenKTesTime-triangleTime);

    System.out.println(Filesystem.getOperatingDirectory());

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
    m_chooseAutos.addOption("Shoot Preload Far Away", "ShootPreloadFarAway");
    m_chooseAutos.addOption("Only Shoot", "OnlyShoot");
    m_chooseAutos.setDefaultOption("Do Nothing", "DoNothing");
    m_chooseAutos.onChange(this::drawSelectedAuto);

    SmartDashboard.putData("Auto Chooser", m_chooseAutos);

    compoundCommands = new CommandBlocks(m_robotDrive, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
        m_pivotSubsystem, m_snuffilatorSubsystem);
    configureCommands();
    configureBindings();

  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    // Some condition that should decide if we want to override rotation
    Optional<Double> angle = m_limelightSubsystem.getNoteTX();
    if (angle.isPresent() && m_robotDrive.m_pathplannerUsingNoteVision) {
      // Return an optional containing the rotation override (this should be a
      // field relative rotation)
      return Optional.of(new Rotation2d(Math
          .toRadians(-angle.get())).plus(m_robotDrive.getPose().getRotation()));
    } else {
      // return an empty optional when we don't want to override the path's
      // rotation
      return Optional.empty();
    }
  }

  private void configureCommands() {
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    NamedCommands.registerCommand("startIntaking", compoundCommands.startIntaking());
    NamedCommands.registerCommand("stopIntaking", compoundCommands.stopIntaking());
    NamedCommands.registerCommand("cancelIfNote", new InstantCommand(() -> {
      m_robotDrive.m_pathPlannerCancelIfNoteSeen = true;
    }));
    NamedCommands.registerCommand("driveToNote", new InstantCommand(() -> {
      m_robotDrive.m_pathplannerUsingNoteVision = true;
    }));

    if (SubsystemConstants.useShooter) {
      NamedCommands.registerCommand("runShooter", new SetShooterCommand(m_shooterSubsystem, null, null));
      NamedCommands.registerCommand("stopShooter", new SetShooterCommand(m_shooterSubsystem, 0.0, 0.0));
    }
  }

  private void configureBindings() {

    if (SubsystemConstants.useDrive) {
      m_driveJoystick.button(ControllerConstants.resetYawButton).whileTrue(new InstantCommand(m_robotDrive::resetYaw)); // Flightstick
      // button
      // 11
      Trigger fieldRelativeTrigger = ControllerConstants.useXbox
          ? new Trigger(m_driveJoystick.axisGreaterThan(2, 0.1))
          : new Trigger(m_driveJoystick.button(ControllerConstants.fieldRelativeButton));
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
      // m_opJoystick.axisGreaterThan(3, 0.1).whileTrue(
      // new SetShooterCommand(m_shooterSubsystem, null, null)
      // .andThen(new InstantCommand(m_shooterSubsystem::stopShooter)));
      m_opJoystick.axisGreaterThan(3, 0.1).whileTrue(
          new ConditionalCommand(
              new SetShooterCommand(m_shooterSubsystem, ShooterConstants.shooterSpeedAmp,
                  ShooterConstants.shooterSpeedAmp),
              new SetShooterCommand(m_shooterSubsystem, null, null),
              () -> {
                return m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition;
              })
              .andThen(new InstantCommand(m_shooterSubsystem::stopShooter)));
      // TODO add trigger if statement
      m_opJoystick.button(3).whileTrue(compoundCommands.fireButtonHold());
      m_driveJoystick.button(ControllerConstants.yuckButton).whileTrue(compoundCommands.yuckButtonHold());
    }
    if (SubsystemConstants.useShooter && SubsystemConstants.usePivot) {
      new Trigger(() -> {
        return m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition;
      })
          .onTrue(new SetShooterCommand(m_shooterSubsystem, ShooterConstants.shooterSpeedAmp,
              ShooterConstants.shooterSpeedAmp))
          .onFalse(new SetShooterCommand(m_shooterSubsystem, null, null));
    }

    if (SubsystemConstants.useClimber) {
      m_driveJoystick.povUp().whileTrue(new StartEndCommand(
          () -> {
            m_climberSubsystem.climbMotorLeft.set(ClimberConstants.climbSpeed);
            m_climberSubsystem.climbMotorRight.set(ClimberConstants.climbSpeed);
          },
          () -> {
            m_climberSubsystem.climbMotorLeft.set(0);
            m_climberSubsystem.climbMotorRight.set(0);
          }, m_climberSubsystem));
      m_driveJoystick.povDown().whileTrue(new StartEndCommand(
          () -> {
            m_climberSubsystem.climbMotorLeft.set(-ClimberConstants.climbSpeed);
            m_climberSubsystem.climbMotorRight.set(-ClimberConstants.climbSpeed);
          },
          () -> {
            m_climberSubsystem.climbMotorLeft.set(0);
            m_climberSubsystem.climbMotorRight.set(0);
          }, m_climberSubsystem));
    }

    if (SubsystemConstants.useIntake) {
      Trigger intakeTrigger = ControllerConstants.useXbox
          ? new Trigger(m_driveJoystick.axisGreaterThan(3, 0.1))
          : new Trigger(m_driveJoystick.button(ControllerConstants.intakeButton));
      intakeTrigger.whileTrue(compoundCommands.intakeButtonHold());
      new Trigger(() -> {
        return m_intakeSubsystem.m_beambreak.beamBroken();
      })
          .onTrue(new InstantCommand(() -> {
            m_driveJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.2);
          }))
          .onFalse(new InstantCommand(() -> {
            m_driveJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          }));
    }

    if (SubsystemConstants.usePivot) {
      m_opJoystick.povRight().whileTrue(new PivotManualCommand(m_pivotSubsystem, PivotConstants.manualSpeed));
      m_opJoystick.povLeft().whileTrue(new PivotManualCommand(m_pivotSubsystem, -PivotConstants.manualSpeed));
      // m_opJoystick.button(7).onTrue(new
      // InstantCommand(m_pivotSubsystem::togglePID));
      m_opJoystick.button(8).onTrue(new InstantCommand(m_shooterSubsystem::stopShooter));
      m_opJoystick.button(1)
          .onTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.subwooferPosition)));
      m_opJoystick.button(2).whileTrue(
          new ParallelCommandGroup(
              new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.ampPosition))));
      m_opJoystick.button(4)
          .whileTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.podiumPosition)));
    }
    if (SubsystemConstants.useDrive && SubsystemConstants.useLimelight) {
      m_driveJoystick.button(ControllerConstants.drivePointedToSpeakerButton)
          .whileTrue(new DrivePointedToSpeakerCommand(m_robotDrive, m_limelightSubsystem, m_driveJoystick));
      m_driveJoystick.axisGreaterThan(ControllerConstants.drivePointedToNoteAxis, 0.1)
          .whileTrue(new DrivePointedToNoteCommand(m_robotDrive, m_limelightSubsystem, m_driveJoystick));
      m_opJoystick.axisGreaterThan(2, 0.1)
          .whileTrue(new PivotMagicCommand(m_pivotSubsystem, m_limelightSubsystem))
          .whileTrue(new OperatorRumbleCommand(m_pivotSubsystem, m_robotDrive, m_limelightSubsystem, m_shooterSubsystem,
              m_opJoystick));
      m_driveJoystick.button(1)
          .whileTrue(new DriveToNoteCommand(m_robotDrive, m_limelightSubsystem, m_intakeSubsystem, m_triggerSubsystem,
              m_driveJoystick));
    }

    if (SubsystemConstants.useSnuffilator) {
      new Trigger(() -> {
        return m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition;
      })
          .onTrue(compoundCommands.moveSnuffilator(true))
          .onFalse(compoundCommands.moveSnuffilator(false));
    }
  }

  // pulls beambreak every millisecond
  public void fastBeamBreakCheckIntake() {
    if (!SubsystemConstants.useIntake) {
      return;
    }
    if (m_intakeSubsystem.beambreakState // TODO: Maybe rename this?
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
    m_triggerSubsystem.beambreakCount = 0;
    if (m_triggerSubsystem.triggerYuck || m_triggerSubsystem.triggerFire) {
      return;
    }
    // NeutralOut neutral = new NeutralOut();
    // neutral.UpdateFreqHz = 1000;
    m_triggerSubsystem.m_triggerMotor.setControl(new NeutralOut());
    // System.out.println("quick stop");
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

  public Command getAutonomousCommand() { // TODO put by auto chooser
    if (m_chooseAutos.getSelected() == "CenterDown") {
      return Autos.CenterDown(compoundCommands, m_robotDrive, m_shooterSubsystem);
    }
    if (m_chooseAutos.getSelected() == "FourPiece") {
      return Autos.FourPiece(compoundCommands, m_robotDrive, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
          m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "FourPieceCenter") {
      return Autos.FourPieceCenter(compoundCommands, m_robotDrive, m_intakeSubsystem, m_shooterSubsystem,
          m_triggerSubsystem, m_pivotSubsystem);
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
      return Autos.ShootPreloadFarAway(compoundCommands, m_robotDrive, m_shooterSubsystem, m_limelightSubsystem,
          m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "OnlyShoot") {
      return Autos.OnlyShoot(compoundCommands, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
          m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "DoNothing") {
      return Autos.DoNothing();
    } else {
      return Autos.DoNothing();
    }
  }

  public void drawSelectedAuto(String selection) {
    m_autoPath = new ArrayList<>();
    String autoFile = "";
    if (selection == "FourPieceCenter") {
      autoFile = "4 Piece Center";
    }
    if (selection == "ThreePieceCenter") {
      autoFile = "3 Piece Center";
    }
    if (selection == "FourPiece") {
      autoFile = "4 Piece Auto";
    }
    if (selection == "BasicAmp") {
      autoFile = "Basic Amp";
    }
    if (selection == "CenterDown") {
      autoFile = "Center Down";
    }
    if (selection == "GrandTheftAuto") {
      autoFile = "Grand Theft Auto";
    }
    if (selection == "IntermediateAmp") {
      autoFile = "Intermediate Amp";
    }
    if (selection == "ShootPreloadFarAway") {
      autoFile = "Shoot Preload Far Away";
    }
    if (autoFile != "") {
      Pose2d pose = PathPlannerAuto.getStaringPoseFromAutoFile(autoFile);
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          pose = GeometryUtil.flipFieldPose(pose);
        }
      }
      m_autoPose.setRobotPose(pose);
      PathPlannerAuto.getPathGroupFromAutoFile(autoFile).forEach(this::drawTrajectory);
      SmartDashboard.putData(m_autoPose);
    }
    m_autoPose.getObject("Auto Path").setPoses(m_autoPath);
  }

  public void drawTrajectory(PathPlannerPath path) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        m_autoPath.addAll(path.flipPath().getPathPoses());
      } else {
        m_autoPath.addAll(path.getPathPoses());
      }
    } else {
      m_autoPath.addAll(path.getPathPoses());
    }
  }
}
