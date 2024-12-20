// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SnuffilatorConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.DrivePointedToNoteCommand;
import frc.robot.commands.DrivePointedToSpeakerCommand;
import frc.robot.commands.DriveToNoteCommand;
import frc.robot.commands.DriveToPoseCommand;
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

import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  private CommandBlocks m_compoundCommands;
  public final Field2d m_autoPose = new Field2d();
  public List<Pose2d> m_autoPath = new ArrayList<>();

  // The robot's subsystems and commands are defined here...
  final PoseEstimatorSubsystem m_poseEstimatorSubsystem = new PoseEstimatorSubsystem(); // TODO add optional
                                                                                        // initializatoin
  final DriveSubsystem m_driveSubsystem = SubsystemConstants.useDrive ? new DriveSubsystem(m_poseEstimatorSubsystem)
      : null;
  private final LimelightSubsystem m_limelightSubsystem = SubsystemConstants.useLimelight
      ? new LimelightSubsystem(m_poseEstimatorSubsystem)
      : null;

  private final ShooterSubsystem m_shooterSubsystem = SubsystemConstants.useShooter ? new ShooterSubsystem()
      : null;
  private final IntakeSubsystem m_intakeSubsystem = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  final PivotSubsystem m_pivotSubsystem = SubsystemConstants.usePivot ? new PivotSubsystem() : null;
  public final ClimbSubsystem m_climberSubsystem = SubsystemConstants.useClimber ? new ClimbSubsystem() : null;
  private final TriggerSubsystem m_triggerSubsystem = SubsystemConstants.useShooter ? new TriggerSubsystem() : null;
  public final SnuffilatorSubsystem m_snuffilatorSubsystem = SubsystemConstants.useSnuffilator
      ? new SnuffilatorSubsystem()
      : null;
  private final TriangleInterpolator m_triangleInterpolator = new TriangleInterpolator(4);
  private final TriangleInterpolator m_feederInterpolator = new TriangleInterpolator(4);
  private final PowerDistribution pdh = new PowerDistribution();

  private final SendableChooser<String> m_chooseAutos = new SendableChooser<>();
  public Command m_autonomousCommand = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // TriangleInterpolator.addDuluthMagic(m_triangleInterpolator);
    // m_triangleInterpolator.addCalibratedPoint(2.0, 7.78, 0, 0, 0.374, 0);
    TriangleInterpolator.addv2Magic(m_triangleInterpolator);
    TriangleInterpolator.addFeederMagic(m_feederInterpolator);
    double startTime = Timer.getFPGATimestamp();
    m_triangleInterpolator.makeTriangles();
    m_feederInterpolator.makeTriangles();
    double triangleTime = Timer.getFPGATimestamp();
    // m_triangleInterpolator.draw("C:\\Users\\Scottie\\Desktop\\Automatons-Code\\AdvantageLogs\\shooterSpeedLeftTestImage.png",
    // 1000,
    // 1000, 0, 8.27, 8.27, 0, 0, 0, 80);
    // m_triangleInterpolator.draw("C:\\Users\\Scottie\\Desktop\\Automatons-Code\\AdvantageLogs\\shooterSpeedRightTestImage.png",
    // 1000,
    // 1000, 0, 8.27, 8.27, 0, 1, 0, 80);
    // m_triangleInterpolator.draw("C:\\Users\\Scottie\\Desktop\\Automatons-Code\\AdvantageLogs\\pivotAngleTestImage.png",
    // 1000, 1000,
    // 0, 8.27, 8.27, 0, 2, 0.3, 0.5);
    // m_triangleInterpolator.draw("C:\\Users\\Scottie\\Desktop\\Automatons-Code\\AdvantageLogs\\headingTestImage.png",
    // 1000, 1000, 0,
    // 8.27, 8.27, 0, 3, (-Math.PI)/2, (Math.PI)/2);

    // m_feederInterpolator.draw("C:\\Users\\Scottie\\Automatons-Code\\Logs\\shooterSpeedLeftFeederImage.png",
    // 2000,
    // 1000, 0, 16.54, 8.27, 0, 0, 0, 80);
    // m_feederInterpolator.draw("C:\\Users\\Scottie\\Automatons-Code\\Logs\\shooterSpeedRightFeederImage.png",
    // 2000,
    // 1000, 0, 16.54, 8.27, 0, 1, 0, 80);
    // m_feederInterpolator.draw("C:\\Users\\Scottie\\Automatons-Code\\Logs\\pivotAngleFeederImage.png",
    // 2000, 1000,
    // 0, 16.54, 8.27, 0, 2, 0.3, 0.5);
    // m_feederInterpolator.draw("C:\\Users\\Scottie\\Automatons-Code\\Logs\\headingFeederImage.png",
    // 2000, 1000, 0,
    // 16.54, 8.27, 0, 3, (-Math.PI)/2, (Math.PI)/2);
    double TenKTesTime = Timer.getFPGATimestamp();
    System.out.println(triangleTime - startTime);
    System.out.println(TenKTesTime - triangleTime);

    System.out.println(Filesystem.getOperatingDirectory());

    pdh.setSwitchableChannel(true); // TODO are we still using this?

    if (SubsystemConstants.useDataManager) {
      DataLogManager.start();
    }

    m_compoundCommands = new CommandBlocks(m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
        m_pivotSubsystem, m_snuffilatorSubsystem, m_limelightSubsystem, m_poseEstimatorSubsystem,
        m_triangleInterpolator, m_feederInterpolator);

    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.setDefaultCommand(

          new RunCommand(
              () -> {
                m_driveSubsystem.driveWithJoystick(m_driveJoystick);
              },
              m_driveSubsystem));
    }

    if (SubsystemConstants.useIntake && SubsystemConstants.useTrigger) {
      m_intakeSubsystem.setDefaultCommand(
          new IntakeStatesCommand(m_intakeSubsystem, m_triggerSubsystem.m_beamBreak));
      m_triggerSubsystem.setDefaultCommand(
          new TriggerStatesCommand(m_triggerSubsystem, m_triggerSubsystem.m_beamBreak));
      m_driveJoystick.button(ControllerConstants.yuckButton).whileTrue(m_compoundCommands.yuckButtonHold());
      Trigger intakeTrigger = ControllerConstants.useXbox
          ? new Trigger(m_driveJoystick.axisGreaterThan(3, 0.1))
          : new Trigger(m_driveJoystick.button(ControllerConstants.intakeButton));
      intakeTrigger.whileTrue(m_compoundCommands.intakeButtonHold());
      m_opJoystick.button(7).whileTrue(
          new StartEndCommand(() -> m_intakeSubsystem.runIntake(-IntakeConstants.intakeSpeed, 0.0, 0.0),
              () -> m_intakeSubsystem.runIntake(0, 0, 0), m_intakeSubsystem))
          .whileTrue(new StartEndCommand(() -> m_triggerSubsystem.intakeButtonPressed(),
              () -> m_triggerSubsystem.intakeButtonReleased()));
    }
    configureAutoSelector();
    configureNamedCommands();
    configureButtonBindings();
  }

  private void configureNamedCommands() {
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    NamedCommands.registerCommand("startIntaking", m_compoundCommands.startIntaking());
    NamedCommands.registerCommand("stopIntaking", m_compoundCommands.stopIntaking());
    NamedCommands.registerCommand("cancelIfNote", new InstantCommand(() -> {
      m_driveSubsystem.m_pathPlannerCancelIfNoteSeen = true;
    }));
    NamedCommands.registerCommand("pointAtNote", new InstantCommand(() -> {
      m_driveSubsystem.m_pathplannerUsingNoteVision = true;
    }));
    NamedCommands.registerCommand("pointAtSpeaker", new InstantCommand(() -> {
      AutoConstants.pathplannerOveridePointToSpeaker = true;
    }));
    NamedCommands.registerCommand("stopOverrideRotations", new InstantCommand(() -> {
      AutoConstants.pathplannerOveridePointToSpeaker = false;
      m_driveSubsystem.m_pathplannerUsingNoteVision = false;
    }));
    NamedCommands.registerCommand("magicPivot", new PivotMagicCommand(
        m_pivotSubsystem, m_limelightSubsystem, m_triangleInterpolator, m_feederInterpolator,
        m_poseEstimatorSubsystem));

    if (SubsystemConstants.useShooter) {
      NamedCommands.registerCommand("runShooter",
          new SetShooterCommand(m_shooterSubsystem, m_poseEstimatorSubsystem, m_triangleInterpolator,
              m_feederInterpolator));
      NamedCommands.registerCommand("stopShooter", new SetShooterCommand(m_shooterSubsystem, 0.0, 0.0));
    }
  }

  private void configureButtonBindings() {

    if (SubsystemConstants.useDrive) {
      // m_opJoystick.button(8)
      //     .whileTrue(new InstantCommand(() -> {
      //       Rotation2d heading = m_poseEstimatorSubsystem.getPose().getRotation();
      //       m_driveSubsystem.resetYaw();
      //       m_driveSubsystem.resetPoseEstimatorHeading(heading);
      //     }));
      m_opJoystick.button(8)
          .whileTrue(new InstantCommand(() -> {
            m_driveSubsystem.resetYaw();
            double angle = (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) ? Math.PI : 0;
            m_driveSubsystem.resetPoseEstimatorHeading(new Rotation2d(angle));
          }));
      // Trigger fieldRelativeTrigger = ControllerConstants.useXbox
      // ? new Trigger(m_driveJoystick.axisGreaterThan(2, 0.1))
      // : new
      // Trigger(m_driveJoystick.button(ControllerConstants.fieldRelativeButton));
      // m_driveJoystick.button(7).whileTrue(new FollowPathCommand(m_robotDrive, "Test
      // Path"));
      // m_driveJoystick.button(8).whileTrue(new FollowPathCommand(m_robotDrive, "Test
      // Path Line"));

      m_driveJoystick.button(ControllerConstants.robotRelativeButton)
          .toggleOnTrue(new InstantCommand(m_driveSubsystem::toggleFieldRelative));

      // m_driveJoystick.button(ControllerConstants.robotRelativeButton)
      // .onTrue(new InstantCommand(() -> m_driveSubsystem.setFieldRelative(false)))
      // .onFalse(new InstantCommand(() -> m_driveSubsystem.setFieldRelative(true)));

      m_driveJoystick.button(ControllerConstants.driveToAmpButton)
          .whileTrue(new RunCommand(() -> {
            double angle = (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) ? 90.0 : 90.0;
            m_driveSubsystem.drivePointedTowardsAngle(m_driveJoystick, Rotation2d.fromDegrees(angle));
          }, m_driveSubsystem));
      // .whileTrue(new DriveToPoseCommand(m_driveSubsystem, "Amp Lineup"));
    }

    // m_driveJoystick.button(ControllerConstants.pointToCornerButton)
    // .whileTrue(new RunCommand(() -> {
    // double angle =
    // (m_limelightSubsystem.getAngleTo(m_poseEstimatorSubsystem.getPose(),
    // LimelightConstants.feedPosition)-180);
    // m_driveSubsystem.drivePointedTowardsAngle(m_driveJoystick,
    // Rotation2d.fromDegrees(angle));
    // }, m_driveSubsystem));

    if (SubsystemConstants.useShooter) {
      m_driveJoystick.button(6).onTrue(
          new ConditionalCommand(
              new SetShooterCommand(m_shooterSubsystem, ShooterConstants.shooterSpeedAmp,
                  ShooterConstants.shooterSpeedAmp),
              new SetShooterCommand(m_shooterSubsystem, m_poseEstimatorSubsystem, m_triangleInterpolator,
                  m_feederInterpolator),
              () -> {
                return m_pivotSubsystem != null && m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition;
              }));
      m_driveJoystick.button(5).onTrue(new SetShooterCommand(m_shooterSubsystem,
          ShooterConstants.preloadSpeedLeft, ShooterConstants.preloadSpeedRight));
      m_driveJoystick.button(5).onFalse(new SetShooterCommand(m_shooterSubsystem, m_poseEstimatorSubsystem,
          m_triangleInterpolator, m_feederInterpolator));
      if (SubsystemConstants.useTrigger) {
        m_opJoystick.button(3).whileTrue(m_compoundCommands.fireButtonHold());
        m_opJoystick.button(3).onTrue(new InstantCommand(() -> {
          DataLogManager.log("Shot taken from:  " + m_poseEstimatorSubsystem.getPose().toString());
        }));
      }
      m_opJoystick.button(2).onTrue(new InstantCommand(m_shooterSubsystem::stopShooter, m_shooterSubsystem));
      m_opJoystick.button(2).onTrue(new InstantCommand(() -> m_snuffilatorSubsystem.move(0), m_snuffilatorSubsystem));
      m_opJoystick.povUp().onTrue(new SetShooterCommand(m_shooterSubsystem));
    }
    if (SubsystemConstants.useShooter && SubsystemConstants.usePivot) {
      m_opJoystick.povDown()
          .onTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(m_pivotSubsystem.getNetworkTestValue())))
          .whileTrue(new OperatorRumbleCommand(m_pivotSubsystem, m_driveSubsystem, m_limelightSubsystem,
              m_shooterSubsystem, m_opJoystick));
      new Trigger(() -> {
        return m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition;
      })
          .onTrue(new SetShooterCommand(m_shooterSubsystem, ShooterConstants.shooterSpeedAmp,
              ShooterConstants.shooterSpeedAmp))
          .onFalse(new SetShooterCommand(m_shooterSubsystem, m_poseEstimatorSubsystem, m_triangleInterpolator,
              m_feederInterpolator));
      m_opJoystick.button(5).whileTrue(new ParallelCommandGroup(
          new SetShooterCommand(m_shooterSubsystem, 45.0, 45.0),
          new InstantCommand(() -> m_pivotSubsystem.setPosition(0.38), m_pivotSubsystem)));

    }

    if (SubsystemConstants.useClimber) {
      m_driveJoystick.povUp().whileTrue(
          new ParallelCommandGroup(
              m_climberSubsystem.climbTo(ClimberConstants.climbSpeed),
              new InstantCommand(() -> {
                m_pivotSubsystem.setPosition(PivotConstants.encoderAt90);
              },
                  m_pivotSubsystem),
              new InstantCommand(() -> {
                m_snuffilatorSubsystem.move(SnuffilatorConstants.snuffilatorOutSpeed);
              })));

      m_driveJoystick.povDown().whileTrue(
          new ParallelCommandGroup(
              m_climberSubsystem.climbTo(-ClimberConstants.climbSpeed),
              new RunCommand(() -> {
                m_climberSubsystem.adjustPivot(m_pivotSubsystem);
              }, m_pivotSubsystem)));

      m_driveJoystick.button(3).whileTrue(m_climberSubsystem.calibrate());
    }

    if (SubsystemConstants.useIntake) {
      new Trigger(() -> {
        return m_intakeSubsystem.m_beambreak.beamBroken();
      }).debounce(0.08, DebounceType.kRising)
          .onTrue(new InstantCommand(() -> {
            m_driveJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.4);
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
      m_opJoystick.button(5)
          .onTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.subwooferPosition),
              m_pivotSubsystem));
      m_driveJoystick.button(6).whileTrue(
          new ParallelCommandGroup(
              new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.ampPosition), m_pivotSubsystem)));
      m_opJoystick.button(4)
          .whileTrue(new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.podiumPosition)));
    }
    if (SubsystemConstants.useDrive && SubsystemConstants.useLimelight) {
      (m_driveJoystick.button(1))
          .whileTrue(
              new ConditionalCommand(
                  m_compoundCommands.fireButtonHold(),
                  new ParallelCommandGroup(
                      new DrivePointedToSpeakerCommand(m_driveSubsystem, m_limelightSubsystem, m_poseEstimatorSubsystem,
                          m_driveJoystick, m_triangleInterpolator, m_feederInterpolator),
                      m_compoundCommands.magicFireGamePieceCommand()),
                      () -> m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition)
                      
                  // new ParallelCommandGroup(
                  //   m_compoundCommands.fireButtonHold(),
                  //   new SetShooterCommand(m_shooterSubsystem, ShooterConstants.shooterSpeedAmp, ShooterConstants.shooterSpeedAmp)

                  );
                  // () -> {
                  //   return m_pivotSubsystem.m_setpoint != PivotConstants.ampPosition;
                  // }));
      // new DrivePointedToSpeakerCommand(m_driveSubsystem, m_limelightSubsystem,
      // m_poseEstimatorSubsystem,
      // m_driveJoystick, m_triangleInterpolator, m_feederInterpolator)); // TODO use
      // pose estimator constant
      m_driveJoystick.axisGreaterThan(ControllerConstants.drivePointedToNoteAxis, 0.1)
          .whileTrue(new DrivePointedToNoteCommand(m_driveSubsystem, m_limelightSubsystem, m_driveJoystick));
      // intake

      // m_opJoystick.axisGreaterThan(2, 0.1)
      // .whileTrue(new PivotMagicCommand(m_pivotSubsystem, m_limelightSubsystem,
      // m_triangleInterpolator,
      // m_feederInterpolator,
      // m_poseEstimatorSubsystem))
      // .whileTrue(
      // new OperatorRumbleCommand(m_pivotSubsystem, m_driveSubsystem,
      // m_limelightSubsystem, m_shooterSubsystem,
      // m_opJoystick)); // TODO change with pose estimator
      m_driveJoystick.axisGreaterThan(ControllerConstants.driveToNoteAxis, 0.1) // TODO change button, and put in if
                                                                                // statement
          .whileTrue(
            new ParallelCommandGroup(
              new DriveToNoteCommand(m_driveSubsystem, m_limelightSubsystem, m_intakeSubsystem, m_triggerSubsystem,
                  m_driveJoystick, () -> {
                    return m_driveJoystick.getRawAxis(2);
                  }),
            new InstantCommand(() -> m_pivotSubsystem.setPosition(PivotConstants.podiumPosition))
            ));
              
    }

    if (SubsystemConstants.useSnuffilator) {
      new Trigger(() -> {
        return m_pivotSubsystem.m_setpoint == PivotConstants.ampPosition;
      })
          .onTrue(m_compoundCommands.moveSnuffilator(true))
          .onFalse(m_compoundCommands.moveSnuffilator(false));
    }

    if (SubsystemConstants.useShooter && SubsystemConstants.usePivot && SubsystemConstants.useDrive) {
      m_opJoystick.button(7).onTrue(new InstantCommand(() -> {
        if (m_poseEstimatorSubsystem.getPose() == null) {
          return;
        }
        Pose2d robotPose = m_poseEstimatorSubsystem.getAlliancePose();
        DataLogManager.log(String.format(
            "m_triangleInterpolator.addCalibratedPoint(%.2f, %.2f, %.1f, %.1f, %.4f, %.4f);",
            robotPose.getX(),
            robotPose.getY(),
            m_shooterSubsystem.shooterTable.getEntry("leftMotor Setpoint").getDouble(ShooterConstants.shooterSpeedLeft),
            m_shooterSubsystem.shooterTable.getEntry("rightMotor Setpoint")
                .getDouble(ShooterConstants.shooterSpeedRight),
            m_pivotSubsystem.getCurrentPosition(),
            robotPose.getRotation().getRadians()));
      }));
    }
  }

  // pulls beambreak every millisecond
  public void fastBeamBreakCheckIntake() {
    if (!SubsystemConstants.useIntake) {
      return;
    }
    if (m_intakeSubsystem.m_isLoaded // TODO: Maybe rename this?
        || (!m_intakeSubsystem.m_isFiring && !m_intakeSubsystem.m_isIntaking && !m_intakeSubsystem.m_isYucking)) {
      return;
    }
    if (!m_triggerSubsystem.m_beamBreak.beamBroken()) {
      return;
    }
    m_intakeSubsystem.m_isLoaded = true;

    if (m_intakeSubsystem.m_isYucking || m_intakeSubsystem.m_isFiring) {
      return;
    }
    m_intakeSubsystem.m_motor.setControl(new NeutralOut());
  }

  public void fastBeamBreakCheckTrigger() {
    if (!SubsystemConstants.useTrigger) {
      return;
    }
    if (m_triggerSubsystem.m_isLoaded
        || (!m_triggerSubsystem.m_isFiring && !m_triggerSubsystem.m_isIntaking && !m_triggerSubsystem.m_isYucking)) {
      return;
    }
    if (!m_triggerSubsystem.m_beamBreak.beamBroken()) {
      return;
    }
    // DataLogManager.log("1ms loop beam break");
    m_triggerSubsystem.m_isLoaded = true;
    m_triggerSubsystem.beambreakCount = 0;
    if (m_triggerSubsystem.m_isYucking || m_triggerSubsystem.m_isFiring) {
      return;
    }
    // NeutralOut neutral = new NeutralOut();
    // neutral.UpdateFreqHz = 1000;
    m_triggerSubsystem.setTrigger(0);
    // DataLogManager.log("1ms loop stopped motor");
    // System.out.println("quick stop");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void resetDriveOffsets() {
    if (SubsystemConstants.useDrive) {
      m_driveSubsystem.resetOffsets();
    }
  }

  public void configureAutoSelector() {
    m_chooseAutos.addOption("Grand Theft Auto", "GrandTheftAuto");
    m_chooseAutos.addOption("Test Path 5", "TestPath5");
    m_chooseAutos.addOption("Shoot Preload Far Away", "ShootPreloadFarAway");
    m_chooseAutos.addOption("Only Shoot", "OnlyShoot");
    m_chooseAutos.addOption("Note Cancel Test", "NoteCancelTest");
    m_chooseAutos.addOption("Amp Center 4 Piece", "AmpCenter4Piece");
    m_chooseAutos.addOption("Middle Alliance 4 Piece", "MidAlliance4Piece");
    m_chooseAutos.addOption("Middle Alliance 4 Piece Skip 1A", "MidAlliance4PieceSkip1A");
    m_chooseAutos.addOption("Source Center 3 Piece", "SourceCenter3Piece");
    m_chooseAutos.setDefaultOption("Do Nothing", "DoNothing");
    m_chooseAutos.onChange(this::drawSelectedAuto);

    SmartDashboard.putData("Auto Chooser", m_chooseAutos);
    m_autonomousCommand = getAutonomousCommand();
  }

  public Command getAutonomousCommand() {
    if (m_chooseAutos.getSelected() == "GrandTheftAuto") {
      return Autos.GrandTheftAuto(m_driveSubsystem);
    }
    if (m_chooseAutos.getSelected() == "TestPath5") {
      return Autos.FiveMeterTest(m_driveSubsystem);
    }
    if (m_chooseAutos.getSelected() == "ShootPreloadFarAway") {
      return Autos.ShootPreloadFarAway(m_compoundCommands, m_driveSubsystem, m_shooterSubsystem, m_limelightSubsystem,
          m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "OnlyShoot") {
      return Autos.OnlyShoot(m_compoundCommands, m_intakeSubsystem, m_shooterSubsystem, m_triggerSubsystem,
          m_pivotSubsystem);
    }
    if (m_chooseAutos.getSelected() == "NoteCancelTest") {
      return Autos.NoteCancelTest(m_compoundCommands, m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem,
          m_limelightSubsystem, m_triggerSubsystem);
    }
    if (m_chooseAutos.getSelected() == "AmpCenter4Piece") {
      return Autos.AmpCenter4Piece(m_compoundCommands, m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem,
          m_pivotSubsystem,
          m_limelightSubsystem, m_triggerSubsystem, m_poseEstimatorSubsystem, m_triangleInterpolator,
          m_feederInterpolator);
    }
    if (m_chooseAutos.getSelected() == "MidAlliance4Piece") {
      return Autos.MiddleAllianceFourPiece(m_compoundCommands, m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem,
          m_pivotSubsystem,
          m_limelightSubsystem, m_triggerSubsystem, m_poseEstimatorSubsystem, m_triangleInterpolator,
          m_feederInterpolator);
    }
    if (m_chooseAutos.getSelected() == "MidAlliance4PieceSkip1A") {
      return Autos.MiddleAllianceFourPieceSkip1A(m_compoundCommands, m_driveSubsystem, m_intakeSubsystem,
          m_shooterSubsystem, m_pivotSubsystem, m_limelightSubsystem, m_triggerSubsystem, m_poseEstimatorSubsystem,
          m_triangleInterpolator, m_feederInterpolator);
    }
    if (m_chooseAutos.getSelected() == "SourceCenter3Piece") {
      return Autos.SourceCenter3Piece(m_compoundCommands, m_driveSubsystem, m_intakeSubsystem, m_shooterSubsystem,
          m_pivotSubsystem,
          m_limelightSubsystem, m_triggerSubsystem, m_poseEstimatorSubsystem, m_triangleInterpolator,
          m_feederInterpolator);
    }
    if (m_chooseAutos.getSelected() == "DoNothing") {
      return Autos.DoNothing();
    } else {
      return Autos.DoNothing();
    }
  }

  public void drawSelectedAuto(String selection) {
    m_autonomousCommand = getAutonomousCommand();
    m_autoPath = new ArrayList<>();
    String autoFile = "";
    if (selection == "GrandTheftAuto") {
      autoFile = "Grand Theft Auto";
    }
    if (selection == "AmpCenter4Piece") {
      autoFile = "Amp Center 4 Piece";
    }
    if (selection == "MidAlliance4Piece") {
      autoFile = "Middle Alliance 4 Piece";
    }
    if (selection == "MidAlliance4PieceSkip1A") {
      autoFile = "Middle Alliance 4 Piece Skip 1A";
    }
    if (selection == "SourceCenter3Piece") {
      autoFile = "Source Center 3 Piece";
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

  public Optional<Rotation2d> getRotationTargetOverride() {
    // Some condition that should decide if we want to override rotation
    Optional<Double> angle = m_limelightSubsystem.getNoteTX();
    if (angle.isPresent() && m_driveSubsystem.m_pathplannerUsingNoteVision) {
      // Return an optional containing the rotation override (this should be a
      // field relative rotation)
      return Optional.of(new Rotation2d(Math
          .toRadians(-angle.get())).plus(m_driveSubsystem.getPose().getRotation()));
    } else if (AutoConstants.pathplannerOveridePointToSpeaker) {
      Pose2d currentPose = m_poseEstimatorSubsystem.getAlliancePose();
      if (currentPose == null) {
        return Optional.empty();
      }
      Optional<double[]> interpolatorData = m_triangleInterpolator.getTriangulatedOutput(currentPose);
      if (interpolatorData.isEmpty()) {
        interpolatorData = m_feederInterpolator.getTriangulatedOutput(currentPose);
      }
      Rotation2d heading;
      if (interpolatorData.isPresent()) {
        heading = new Rotation2d(interpolatorData.get()[3]);
      } else {
        heading = new Rotation2d(Math
            .toRadians(m_limelightSubsystem.getAngleTo(currentPose, LimelightConstants.aprilTagList[7]))
            + Math.PI);
      }
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        heading = GeometryUtil.flipFieldRotation(heading);
      }
      return Optional.of(heading);

    } else {
      // return an empty optional when we don't want to override the path's
      // rotation
      return Optional.empty();
    }
  }
}
