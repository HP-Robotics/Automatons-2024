// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;

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

  public static class VisionConstants {
    public static String kcameraName = "Arducam_Global_Shutter";
    public static boolean kHasVision = true;
    public static Transform2d leftTrans = new Transform2d(new Translation2d(0.35 + 0.4 + .12, 0.6096 - 0.05),
        new Rotation2d(0));
    public static Transform2d rightTrans = new Transform2d(new Translation2d(0.35 + 0.4 + 0.12, -0.6096 - 0.05),
        new Rotation2d(0));
    public static Transform2d centerTrans = new Transform2d(new Translation2d(0.35 + 0.4 + .12, 0), new Rotation2d(0));
    public static Transform2d cameraToRobot = new Transform2d(new Translation2d(-0.3302, 0.0889),
        new Rotation2d(Math.PI));

    public static double acceleration = 1;
    public static double velocity = 1;
    //0.0.05 is distance of outside of bumper, 2 inches, -0.3302 camera to robot
    //0.41 is the beilived distance from April Tags to the outside
  }

  public static String autonomousMode = "NotVision";

  public static class OperatorConstants { // For operator button bindings, and other stuff directly related to the
                                          // operator
    public static final int kDriverControllerPort = 0;
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = true;
    public static final boolean useDataManger = true;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 5.0; // meters per second 
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second  // TODO MENTOR: is this a good turn speed?
    public static final double kSlowSpeed = 2.0;
    public static final double kSlowAngularspeed = Math.PI / 2; // 1/4 rotation per second  // TODO MENTOR: is this a good turn speed?

    public static final double kWheelRadius = 0.0508 * (218.5 / 225.6); //219 /225.6 is a fudge factor
    public static final double kEncoderResolution = 1.0;

    public static final double driveGearRatio = 6.75;
    public static final double turningGearRatio = 15.429;

    public final static Translation2d kFrontLeftLocation = new Translation2d(0.308, 0.308);
    public final static Translation2d kFrontRightLocation = new Translation2d(0.308, -0.308);
    public final static Translation2d kBackLeftLocation = new Translation2d(-0.308, 0.308);
    public final static Translation2d kBackRightLocation = new Translation2d(-0.308, -0.308);

    public final static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackRightLocation, kBackLeftLocation);


        
    public static final double drivekP = 0.0003002932551319648; //old value: 0.0015
    public static final double drivekI = 0.02001955034213099; //old value: 0.0001
    public static final double drivekD = 0.00010009775171065493; //old value: 0.5 
    public static final double drivekF = 0.009609384164222873; //old value: 0.048
    public static final double drivekAllowableError = 50;
    public static final double drivekMaxIntegralAccumulation = 20000; // TODO: a guess, finetune later
    public static final double drivekIntegralZone = 300; //TODO: a guess, finetune later

    public static final double turningkP = 2.0;
    public static final double turningkI = 0.01;
    public static final double turningkD = 0;
    public static final double turningkAllowableError = 50.0/2048;



    public static final double encoderTolerance = 0.01;

  }

  public static class RobotConstants { // For robot values that remain the same, such as max speed
    public static final double swerveOffsetFL = 0.98;
    public static final double swerveOffsetFR = 0.708;
    public static final double swerveOffsetBR = 0.74; // Fill in later (not precise) >: (
    public static final double swerveOffsetBL = 0.55;
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0; // TODO MENTOR: And note that we don't use them...
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxChargeStationVelocity = 2.5;
    public static final double kMaxChargeStationAcceleration = 2;
    public static final double kFastAutoVelocity = 4.5;
    public static final double kfastAutoAcceleration = 3.0;
    public static final double kMaxAutoVelocity = 3;
    public static final double kMaxAutoAcceleration = 3;

    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 20; //5
    public static final double kIXController = 0.2; //0.04
    public static final double kDXController = 0.0;
    public static final double kIYController = 0.2; //0.04
    public static final double kPYController = 20; //5
    public static final double kPThetaController = 3;
    public static final double kIThetaController = 0.05;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double fieldLength = 16.54;
  }

}
