/// Copyright (c) FIRST and other WPILib contributors.
//\ Open Source Software; you can modify and/or share it under the terms of
//     the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.*;
import frc.robot.SwerveModule;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  // BIG BONGO 7
  private final SwerveModule m_frontLeft = new SwerveModule(IDConstants.FLDriveMotorID,
      IDConstants.FLTurningMotorID, PortConstants.FLAbsEncoder, DriveConstants.absEncoderForwardFL, "FL");
  // BIG BONGO 2
  private final SwerveModule m_frontRight = new SwerveModule(IDConstants.FRDriveMotorID,
      IDConstants.FRTurningMotorID, PortConstants.FRAbsEncoder, DriveConstants.absEncoderForwardFR, "FR");
  // BIG BONGO 1
  private final SwerveModule m_backRight = new SwerveModule(IDConstants.BRDriveMotorID,
      IDConstants.BRTurningMotorID, PortConstants.BRAbsEncoder, DriveConstants.absEncoderForwardBR, "BR");
  // BIG BONGO 3
  private final SwerveModule m_backLeft = new SwerveModule(IDConstants.BLDriveMotorID,
      IDConstants.BLTurningMotorID, PortConstants.BLAbsEncoder, DriveConstants.absEncoderForwardBL, "BL");

  private SwerveModuleState[] m_swerveModuleStates = {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };

  public boolean m_fieldRelative = true;

  public final Field2d m_field = new Field2d();
  public final Field2d m_currentPose = new Field2d();
  public final Field2d m_targetPose = new Field2d();

  private final Pigeon2 m_pGyro = new Pigeon2(IDConstants.PigeonID, "CANivore");

  private StatusSignal m_pGyroPitch = m_pGyro.getPitch();
  private StatusSignal m_pGyroYaw = m_pGyro.getYaw();
  private StatusSignal m_pGyroRoll = m_pGyro.getRoll();

  SwerveDriveOdometry m_odometry;
  PIDController rotationController;
  PoseEstimatorSubsystem m_poseEstimator;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable driveTrainTable = inst.getTable("drive-train");
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");
  StructPublisher<Pose2d> drivePublisher;

  public DriveSubsystem(PoseEstimatorSubsystem poseEstimator) {
    m_poseEstimator = poseEstimator;
    m_pGyro.setYaw(0);
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw().getValue()));
    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()

        });
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      m_currentPose.setRobotPose(pose); // TODO: not working, AdvantageScope might say how to fix
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_targetPose.setRobotPose(pose);
    });

    SmartDashboard.putData("Field", m_field);
    rotationController = new PIDController(DriveConstants.turningControllerkP, DriveConstants.turningControllerkI,
        DriveConstants.turningControllerkD);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    drivePublisher = poseEstimatorTable.getStructTopic("Drive Pose", Pose2d.struct).publish();

  }

  @Override
  public void periodic() {
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw().getValue()));
    // Update the odometry in the periodic block
    m_odometry.update(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        });

    m_field.setRobotPose(getPose());
    drivePublisher.set(getPose());
    driveTrainTable.putValue("Robot x", NetworkTableValue.makeDouble(m_odometry.getPoseMeters().getX()));
    driveTrainTable.putValue("Robot y", NetworkTableValue.makeDouble(m_odometry.getPoseMeters().getY()));
    driveTrainTable.putValue("Robot theta",
        NetworkTableValue.makeDouble(m_odometry.getPoseMeters().getRotation().getDegrees()));

    // TODO investigate why this takes so long
    m_frontLeft.updateShuffleboard();
    m_frontRight.updateShuffleboard();
    m_backRight.updateShuffleboard();
    m_backLeft.updateShuffleboard();

    BaseStatusSignal.refreshAll(m_pGyroPitch, m_pGyroYaw, m_pGyroRoll);
    driveTrainTable.putValue("Pigeon Pitch", NetworkTableValue.makeDouble(m_pGyroPitch.getValueAsDouble()));
    driveTrainTable.putValue("Pigeon Yaw", NetworkTableValue.makeDouble(m_pGyroYaw.getValueAsDouble()));
    driveTrainTable.putValue("Pigeon Roll", NetworkTableValue.makeDouble(m_pGyroRoll.getValueAsDouble()));

    // m_poseEstimator.updatePoseEstimator(pigeonYaw,new SwerveModulePosition[] {
    // m_frontLeft.getPosition(),
    // m_frontRight.getPosition(),
    // m_backRight.getPosition(),
    // m_backLeft.getPosition()
    // });

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the ro bot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw().getValue()));

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeonYaw)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void driveWithJoystick(CommandJoystick joystick) {
    drive(
        Math.signum(joystick.getRawAxis(1))

            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(1),
                ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
            * -1 * DriveConstants.kMaxSpeed,
        Math.signum(joystick.getRawAxis(0))
            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(0),
                ControllerConstants.driveJoystickDeadband), ControllerConstants.driveJoystickExponent)
            * -1 * DriveConstants.kMaxSpeed,
        MathUtil.applyDeadband(ControllerConstants.getRotation(joystick), ControllerConstants.driveJoystickDeadband)
            * -1
            * DriveConstants.kMaxAngularSpeed,
        m_fieldRelative);
  }

  public void drivePointedTowardsAngle(CommandJoystick joystick, Rotation2d targetAngle) {
    double rot = rotationController.calculate(m_odometry.getPoseMeters().getRotation().getRadians(),
        targetAngle.getRadians());
    drive(
        Math.signum(joystick.getRawAxis(1))
            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(1),
                ControllerConstants.driveJoystickDeadband), 2)
            * -1 * DriveConstants.kMaxSpeed,
        Math.signum(joystick.getRawAxis(0))
            * Math.pow(MathUtil.applyDeadband(joystick.getRawAxis(0),
                ControllerConstants.driveJoystickDeadband), 2)
            * -1 * DriveConstants.kMaxSpeed,
        rot * 1 * DriveConstants.kMaxAngularSpeed,
        true);

    driveTrainTable.putValue("Rotation Current Angle",
        NetworkTableValue.makeDouble(m_odometry.getPoseMeters().getRotation().getDegrees()));
    driveTrainTable.putValue("Rotation Target Angle", NetworkTableValue.makeDouble(targetAngle.getDegrees()));
    driveTrainTable.putValue("Rotation Power Input", NetworkTableValue.makeDouble(rot));

    driveTrainTable.putValue("Rotation Controller P",
        NetworkTableValue.makeDouble(rotationController.getP() * rotationController.getPositionError()));
    driveTrainTable.putValue("Rotation Controller Position Error",
        NetworkTableValue.makeDouble(rotationController.getPositionError()));
    driveTrainTable.putValue("Rotation Controller Setpoint",
        NetworkTableValue.makeDouble(rotationController.getSetpoint()));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public ChassisSpeeds getCurrentspeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    this.m_swerveModuleStates = swerveModuleStates;
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_backLeft.setDesiredState(swerveModuleStates[3]);
  }

  public void setFieldRelative(boolean isTrue) {
    m_fieldRelative = isTrue;
  }

  public void initializePoseEstimator(Pose2d pose) {
    m_poseEstimator.createPoseEstimator(DriveConstants.kDriveKinematics,
        new Rotation2d(Math.toRadians(m_pGyro.getYaw().getValue())), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        }, pose);

  }

  public void resetOffsets() { // Turn encoder offset
    m_frontLeft.resetOffset();
    m_frontRight.resetOffset();
    m_backRight.resetOffset();
    m_backLeft.resetOffset();
  }

  public void resetOdometry(Pose2d pose) {
    if (pose == null) {
      return;
    }
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw().getValue()));
    m_odometry.resetPosition(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        },
        pose);
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
  }

  /**
   * Resets robot's conception of field orientation
   */
  public void resetYaw() {
    m_pGyro.setYaw(0);
  }
}
