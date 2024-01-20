/// Copyright (c) FIRST and other WPILib contributors.
//\ Open Source Software; you can modify and/or share it under the terms of
//     the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;
import frc.robot.SwerveModule;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  // BIG BONGO 7
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.FLDriveMotorID,
      DriveConstants.FLTurningMotorID, DriveConstants.FLAbsEncoder, DriveConstants.absEncoderForwardFL, "FL");
  // BIG BONGO 2
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.FRDriveMotorID,
      DriveConstants.FRTurningMotorID, DriveConstants.FRAbsEncoder, DriveConstants.absEncoderForwardFR, "FR");
  // BIG BONGO 1
  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.BRDriveMotorID,
      DriveConstants.BRTurningMotorID, DriveConstants.BRAbsEncoder, DriveConstants.absEncoderForwardBR, "BR");
  // BIG BONGO 3
  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.BLDriveMotorID,
      DriveConstants.BLTurningMotorID, DriveConstants.BLAbsEncoder, DriveConstants.absEncoderForwardBL, "BL");

  private SwerveModuleState[] m_swerveModuleStates = {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };

  public boolean m_fieldRelative = true;
  public boolean m_allowVisionUpdates = false;

  public final Field2d m_field = new Field2d();

  // Duty Encoders may have the wrong values

  private final Pigeon2 m_pGyro = new Pigeon2(DriveConstants.PigeonID);

  SwerveDriveOdometry m_odometry;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelightTable = inst.getTable("limelight-prada");
  NetworkTable driveTrainTable = inst.getTable("drive-train");
  NetworkTable pipeline = inst.getTable("SmartDashboard");
  NetworkTableEntry gamePieceX = limelightTable.getEntry("tx");

  public int m_gamePieceSightCounter = 0;

  public DriveSubsystem() {
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

    SmartDashboard.putData("Field", m_field);
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
    driveTrainTable.putValue("Robot x", NetworkTableValue.makeDouble(m_odometry.getPoseMeters().getX()));
    driveTrainTable.putValue("Robot y", NetworkTableValue.makeDouble(m_odometry.getPoseMeters().getY()));
    driveTrainTable.putValue("Robot theta",
        NetworkTableValue.makeDouble(m_odometry.getPoseMeters().getRotation().getDegrees()));

    m_frontLeft.updateShuffleboard();
    m_frontRight.updateShuffleboard();
    m_backRight.updateShuffleboard();
    m_backLeft.updateShuffleboard();

    driveTrainTable.putValue("Pigeon Pitch", NetworkTableValue.makeDouble(m_pGyro.getPitch().getValue()));
    driveTrainTable.putValue("Pigeon Yaw", NetworkTableValue.makeDouble(m_pGyro.getYaw().getValue()));
    driveTrainTable.putValue("Pigeon Roll", NetworkTableValue.makeDouble(m_pGyro.getRoll().getValue()));

    m_field.setRobotPose(m_odometry.getPoseMeters());

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

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public ChassisSpeeds getCurrentspeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  public double getPoseX() {
    return m_odometry.getPoseMeters().getX();
  }

  public double getPoseY() {
    return getPose().getY();
  }

  public Rotation2d getPoseRot() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    this.m_swerveModuleStates = swerveModuleStates;
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_backLeft.setDesiredState(swerveModuleStates[3]);

  }

  public void forceRobotRelative() {
    m_fieldRelative = false;
  }

  public void forceFieldRelative() {
    m_fieldRelative = true;
  }

  public void resetOffsets() {
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
  }

/** Resets robot's conception of field orientation
 */
  public void resetYaw() {
    m_pGyro.setYaw(0);
  }

}
