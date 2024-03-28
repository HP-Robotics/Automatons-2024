// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/* The Constants class provides a convenient place for teams to hold robot-wide
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

  public static class SubsystemConstants {
    public static final boolean useDrive = true;
    public static final boolean useIntake = true;
    public static final boolean useShooter = true;
    public static final boolean useDataManager = true;
    public static final boolean useLimelight = true;
    public static final boolean usePivot = true;
    public static final boolean useClimber = true;
    public static final boolean useTrigger = true;
    public static final boolean useSnuffilator = true;
  }

  public static class ControllerConstants {
    public static final boolean useXbox = true;

    public static final int kOperatorControllerPort = 0;
    public static final int kDriverControllerPort = 1;
    public static final double driveJoystickDeadband = useXbox ? 0.15 : 0.15;
    public static final double turnJoystickDeadband = useXbox ? 0.1 : 0.1;

    public static final double driveJoystickExponent = useXbox ? 2 : 2;

    public static final int resetYawButton = useXbox ? 7 : 11;
    public static final int fieldRelativeButton = useXbox ? 8 : 8;
    public static final int yuckButton = useXbox ? 4 : 2;
    public static final int climberButton = useXbox ? 10 : 10;
    public static final int intakeButton = useXbox ? 0 : 1;
    public static final int drivePointedToSpeakerButton = useXbox ? 6 : 0;
    public static final int drivePointedToNoteButton = useXbox ? 5 : 0;
    public static final int driveToNoteAxis = useXbox ? 2 : 0;

    // TODO: Add operator joystick constants

    public static double getRotation(CommandJoystick stick) {
      if (useXbox) {
        return stick.getRawAxis(4);
      } else {
        return stick.getRawAxis(2);
        // if (stick.povLeft().getAsBoolean()) {
        // return -0.5;
        // }
        // else if (stick.povRight().getAsBoolean()) {
        // return 0.5;
        // }
        // return 0;
      }
    }
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 4.4; // meters per second 
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second //auto is 540
    public static final double kSlowSpeed = 2.0;
    public static final double kSlowAngularspeed = Math.PI / 2; // 1/4 rotation per second

    public static final double kWheelRadius = 0.0508 * (218.5 / 225.6); // This is a fudge factor
    public static final double kEncoderResolution = 1.0;

    public static final double driveGearRatio = 6.75;
    public static final double turningGearRatio = 540/35;

    public static final Translation2d kFrontLeftLocation = new Translation2d(0.308 - 0.038, 0.308);
    public static final Translation2d kFrontRightLocation = new Translation2d(0.308 - 0.038, -0.308);
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.308, 0.308);
    public static final Translation2d kBackRightLocation = new Translation2d(-0.308, -0.308);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
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
    public static final double turningControllerTolerance = Math.toRadians(2);

    // Absolute encoder values that make the wheels point forward
    public static final double absEncoderForwardFL = 0.98;
    public static final double absEncoderForwardFR = 0.719; //.708
    public static final double absEncoderForwardBR = 0.74;
    public static final double absEncoderForwardBL = 0.55;

    public static final HolonomicPathFollowerConfig holonomicConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        4.5,
        0.4, // Distance from robot center to furthest module.
        new ReplanningConfig());

    public static final double currentMax = 40.0;
    public static final double currentMin = -40.0;

    public static final double rampTimeTo300s = 0.01;

    public static final double currentLimit = 40.0;
    public static final double currentThreshold = 40.0;
    public static final double currentTimeThreshold = 0.04;

    public static final double driveToNoteSpeed = 0.5;
  }

  public static final class LimelightConstants {
    public static final double inToM = 0.0254;
    public static final Pose2d aprilTagList[] = { // 0 is empty, april tag number is that number in list
      new Pose2d(),
      new Pose2d(593.68 * inToM, 9.68 * inToM, new Rotation2d(Math.PI * 2/3)),    // 1
      new Pose2d(637.21 * inToM, 34.79 * inToM, new Rotation2d(Math.PI * 2/3)),   // 2
      new Pose2d(652.73 * inToM, 196.17 * inToM, new Rotation2d(Math.PI)),        // 3
      new Pose2d(652.73 * inToM, 218.42 * inToM, new Rotation2d(Math.PI)),        // 4
      new Pose2d(578.77 * inToM, 323.00 * inToM, new Rotation2d(Math.PI * 3/2)),  // 5
      new Pose2d(72.5 * inToM, 323.00 * inToM, new Rotation2d(Math.PI * 3/2)),    // 6
      new Pose2d(-1.5 * inToM, 218.42 * inToM, new Rotation2d(0)),          // 7
      new Pose2d(-1.5 * inToM, 196.17 * inToM, new Rotation2d(0)),          // 8
      new Pose2d(14.02 * inToM, 34.79 * inToM, new Rotation2d(Math.PI/3)),        // 9
      new Pose2d(57.54 * inToM, 9.68 * inToM, new Rotation2d(Math.PI/3)),         // 10
      new Pose2d(468.69 * inToM, 146.19 * inToM, new Rotation2d(Math.PI * 5/3)),  // 11
      new Pose2d(468.69 * inToM, 177.10 * inToM, new Rotation2d(Math.PI/3)),      // 12
      new Pose2d(441.74 * inToM, 161.62 * inToM, new Rotation2d(Math.PI)),        // 13
      new Pose2d(209.48 * inToM, 161.62 * inToM, new Rotation2d(0)),        // 14
      new Pose2d(182.73* inToM, 177.10 * inToM, new Rotation2d(Math.PI * 2/3)),   // 15
      new Pose2d(182.73 * inToM, 146.19 * inToM, new Rotation2d(Math.PI * 4/3))   // 16
    };

    public static double allowableNoteAngleError = 2.5;
    public static double allowableSpeakerDistanceError = 2.5; // Make these good
  }

  public static final class PoseEstimatorConstants {
    public static final Matrix<N3,N1> statesStandardDev = VecBuilder.fill(0.001,0.001,0.005);
    public static final double visionXStandardDev = 0.01;
    public static final double visionYStandardDev = 0.01;
    public static final double visionHeadingStandardDev = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0;
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

    public static final double additionalIntakeTime = 0.5;
    public static final double additionalShootTime = 0.3;

    public static boolean pathplannerOveridePointToSpeaker = false; 

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double fieldLength = 16.54;
  }

  public static class IntakeConstants {
    public static final double intakeSpeed = -0.45;
    public static final double vanguardSpeed = -0.25;
  }

  public static class SnuffilatorConstants {
    public static final int stallCurrentLimit = 3;
    public static final int freeCurrentLimit = 5;

    public static final double snuffilatorInSpeed = 0.15;
    public static final double snuffilatorOutSpeed = 0.24;
  }

  public static class ShooterConstants {

    public static final double shooterSpeedLeft = 50; // 50
    public static final double shooterSpeedRight = 50; // 50
    public static final double shooterSpeedAmp = 12.5; // 15

    public static final double shooterMotorskP = 9;
    public static final double shooterMotorskI = 4; 
    public static final double shooterMotorskD = 0.0;
    public static final double shooterMotorskV = 0.08;
    public static final double shooterMotorsKS = 6;


    public static final double errorThreshold = 2.0;

    public static final double currentLimit = 25.0;
    public static final double currentThreshold = 25.0;
    public static final double currentTimeThreshold = 0.04;

    public static final double rampTimeTo300s = 0.4;
  }

  public static class ClimberConstants {
    public static final double climbSpeed = 1; //TODO: Decide this (in RPM, so 500?)


    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIz = 0.0;
    public static final double kFF = 0.0;

    public static final double kMinOutput = 0.0;
    public static final double kMaxOutput = 0.0;
  }

  public static class TriggerConstants {
    public static final double triggerSpeed = 0.3;
    public static final double yuckSpeed = -0.2;

    public static final double triggerkV = 0;
    public static final double triggerkP = 0;
    public static final double triggerkI = 0;
    public static final double triggerkD = 0;

  }

  public static class PivotConstants {
    public static final double kP = 40;
    public static final double kI = 5;
    public static final double kD = 2;
    public static final double kG = -0.27;
    public static final boolean startWithPID = true;
    
    public static final double[] magicConstants = { 0.0, -0.0219711, 0.437724 }; 
    
    // new line: { 0.0, -0.0245526, 0.461092 };
    // new quadratic: { 0.0025323, -0.0378195, 0.476864 };
    // old line: { 0.0, -0.0219711, 0.437724 };

    public static final double manualSpeed = 0.1;

    public static final double setpointChangeSpeed = 0.2;
    public static final double encoderAt90 = 0.518;

    public static double degreesToEncoder(double degrees) {
      return (degrees - 90) / 360 + PivotConstants.encoderAt90;
    }

    public static final double currentLimit = 40.0;
    public static final double currentThreshold = 40.0;
    public static final double currentTimeThreshold = 0.04;

    public static final double rampTimeTo300s = 10;

    public static final double subwooferPosition = degreesToEncoder(58.3); // 0.43
    public static final double ampPosition = degreesToEncoder(119.5 + 3); // 0.6
    public static final double podiumPosition = degreesToEncoder(42.12 - 3); // 0.385
    public static final double note2Position = degreesToEncoder(45.72 - 3.25); // 0.395
    public static final double note1_3Position = degreesToEncoder(42.12); // 0.385
    public static final double noteA1Position = 0.3722;
    public static final double noteA3Position = 0.3772;
    public static final double preloadFarAwayPosition = 0.363;
    public static final double minimumPosition = 0.2;
    public static final double maximumPosition = 0.7; 
  }

  public static class IDConstants {
    // Drive is 20s
    public static final int FLDriveMotorID = 28;
    public static final int FRDriveMotorID = 22;
    public static final int BRDriveMotorID = 20;
    public static final int BLDriveMotorID = 24;

    public static final int FLTurningMotorID = 29;
    public static final int FRTurningMotorID = 23;
    public static final int BRTurningMotorID = 21;
    public static final int BLTurningMotorID = 25;

    // Intake is 10s
    public static final int intakeMotorID = 10;
    public static final int vanguardLeftID = 11;
    public static final int vanguardRightID = 12;

    // Shooter is 30s
    public static final int leftMotorID = 30;
    public static final int rightMotorID = 31;
    public static final int triggerMotorID = 32;

    // Pivot is 40s
    public static final int rightPivotID = 40;
    public static final int leftPivotID = 41;

    //Climber is 50s
    public static final int climbMotorRightID = 50;
    public static final int climbMotorLeftID = 51;
    
    public static final int PigeonID = 57;

    // Snuffilator is 60s
    public static final int snuffilatorID = 60;
  }

  public static class PortConstants {
    public static final int FLAbsEncoder = 14;
    public static final int FRAbsEncoder = 12;
    public static final int BRAbsEncoder = 11;
    public static final int BLAbsEncoder = 13;

    public static final int pivotAbsEncoderID = 16;

    public static final int TriggerBeamBreak = 9; // Beam Break port
    public static final int IntakeBeamBreak = 7;
  }

}
