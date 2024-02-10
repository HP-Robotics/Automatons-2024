// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 0;
    public static final int kDriverControllerPort = 1;
    public static final double driveJoystickDeadband = 0.15;
    public static final double turnJoystickDeadband = 0.1;
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = true;
    public static final boolean useIntake = true;
    public static final boolean useShooter = false;
    public static final boolean useDataManger = true;
    public static final boolean useLimelight = false;
    public static final boolean usePivot = false;
    public static final boolean useClimber = false; //TODO check if these work
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 4.0; // meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second 
    public static final double kSlowSpeed = 2.0;
    public static final double kSlowAngularspeed = Math.PI / 2; // 1/4 rotation per second 

    public static final double kWheelRadius = 0.0508 * (218.5 / 225.6); // This is a fudge factor
    public static final double kEncoderResolution = 1.0;

    public static final double driveGearRatio = 6.75;
    public static final double turningGearRatio = 15.429;

    public final static Translation2d kFrontLeftLocation = new Translation2d(0.308-0.038, 0.308);
    public final static Translation2d kFrontRightLocation = new Translation2d(0.308-0.038, -0.308);
    public final static Translation2d kBackLeftLocation = new Translation2d(-0.308, 0.308);
    public final static Translation2d kBackRightLocation = new Translation2d(-0.308, -0.308);

    public final static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackRightLocation, kBackLeftLocation);

    public static final double drivekP = 5;
    public static final double drivekI = 10;
    public static final double drivekD = 0;
    public static final double drivekF = 0;

    public static final double turningkP = 1.8;
    public static final double turningkI = 1;
    public static final double turningkD = 0.008;

    public static final double turningControllerkP = 1;
    public static final double turningControllerkI = 0.0;
    public static final double turningControllerkD = 0.0;
    
    // Absolute encoder values that make the wheels point forward
    public static final double absEncoderForwardFL = 0.98;
    public static final double absEncoderForwardFR = 0.708;
    public static final double absEncoderForwardBR = 0.74;
    public static final double absEncoderForwardBL = 0.55;
  }

  public static final class LimelightConstants {
    public static final Pose2d aprilTag7 = new Pose2d(-1.5 * 0.0254, 218.42 * 0.0254, new Rotation2d(0));
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0;//TODO look at these
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kFastAutoVelocity = 4.5;
    public static final double kfastAutoAcceleration = 3.0;
    public static final double kMaxAutoVelocity = 3;
    public static final double kMaxAutoAcceleration = 3;

    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 20;
    public static final double kIXController = 0.2;
    public static final double kDXController = 0.0;
    public static final double kIYController = 0.2;
    public static final double kPYController = 20;
    public static final double kPThetaController = 3;
    public static final double kIThetaController = 0.05;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double fieldLength = 16.54;
  }

  public static class IntakeConstants {
    public static final double intakeSpeed = -0.95;
  }

  public static class ShooterConstants {
    public static final double shooterSpeed1 = 0;
    public static final double shooterSpeed2 = 0;

    public static final double motor1kP = 0.4;//TODO Rename
    public static final double motor1kI = 0.01;
    public static final double motor1kD = 0;
    public static final double motor1kV = 0.12;

  }

  public static class IDConstants {
    //Drive is 20s
    public final static int FLDriveMotorID = 28;
    public final static int FRDriveMotorID = 22;
    public final static int BRDriveMotorID = 20;
    public final static int BLDriveMotorID = 24;

    public final static int FLTurningMotorID = 29;
    public final static int FRTurningMotorID = 23;
    public final static int BRTurningMotorID = 21;
    public final static int BLTurningMotorID = 25;

    //Intake is 10s
    public static final int intakeMotorID = 10;

    //Shooter is 30s
    public static final int shooterMotor1ID = 30;
    public static final int shooterMotor2ID = 31;

    //Pivot is 40s
    public static final int rightPivotID = 40;
    public static final int leftPivotID = 41;

    //Climber is 50s
    public final static int climbMotorID = 50; //TODO choose ID number
    
    public final static int PigeonID = 57;
    
    public final static int FLAbsEncoder = 14;
    public final static int FRAbsEncoder = 12;
    public final static int BRAbsEncoder = 11;
    public final static int BLAbsEncoder = 13;
    
    public final static int pivotAbsEncoderID = 0;
  }

  public static class ClimberConstants {
    public final static double climbSpeed = 0.4;
  }


  public static class PivotConstants {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final boolean startWithPID = false;

    public static final double manualSpeed = 0.1;
    public static final double position1 = 0.0;

    public static final double setpointChangeSpeed = 1;
  }
}
