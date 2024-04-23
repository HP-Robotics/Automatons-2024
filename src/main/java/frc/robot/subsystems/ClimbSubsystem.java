// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PivotConstants;

public class ClimbSubsystem extends SubsystemBase {
  public CANSparkMax climbMotorLeft = new CANSparkMax(IDConstants.climbMotorLeftID, MotorType.kBrushless);
  public CANSparkMax climbMotorRight = new CANSparkMax(IDConstants.climbMotorRightID, MotorType.kBrushless);
  SparkPIDController climbController;
  RelativeEncoder m_encoder;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable climberTable = inst.getTable("climber-table"); // TODO: Make these names consistant (I like table)
  public boolean climbing = true;
  private double m_leftOffset;
  private double m_rightOffset;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    // climbMotor.getConfigurator().apply(config); // TODO: Is there a configuration
    // for CANSparkMax?
    climbMotorLeft.restoreFactoryDefaults();
    climbMotorRight.restoreFactoryDefaults();

    // climbController.setP(ClimberConstants.kP);
    // climbController.setI(ClimberConstants.kI);
    // climbController.setD(ClimberConstants.kD);
    // climbController.setIZone(ClimberConstants.kIz);
    // climbController.setFF(ClimberConstants.kFF);
    // climbController.setOutputRange(ClimberConstants.kMinOutput,
    // ClimberConstants.kMaxOutput);

    // climbController.setReference(500.0, CANSparkMax.ControlType.kPosition);

    // climbMotorRight.follow(climbMotorLeft,true);
    climbMotorRight.setInverted(true);
    climbMotorLeft.setSoftLimit(SoftLimitDirection.kForward, (float) ClimberConstants.topPosition); //to change, go to the reset calibrations
    climbMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, (float) ClimberConstants.bottomPosition);
    climbMotorRight.setSoftLimit(SoftLimitDirection.kForward, (float) ClimberConstants.topPosition);
    climbMotorRight.setSoftLimit(SoftLimitDirection.kReverse, (float) ClimberConstants.bottomPosition);

    climbMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
    climbMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    climbMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, false);
    climbMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);

    climbMotorLeft.setIdleMode(IdleMode.kBrake);
    climbMotorRight.setIdleMode(IdleMode.kBrake);

    climbMotorLeft.getEncoder().setPosition(0);
    climbMotorRight.getEncoder().setPosition(0);

    climbMotorLeft.burnFlash();
    climbMotorRight.burnFlash();

    climberTable.putValue("Climber kP", NetworkTableValue.makeDouble(ClimberConstants.kP));
    climberTable.putValue("Climber kI", NetworkTableValue.makeDouble(ClimberConstants.kI));
    climberTable.putValue("Climber kD", NetworkTableValue.makeDouble(ClimberConstants.kD));
    climberTable.putValue("Climber kIz", NetworkTableValue.makeDouble(ClimberConstants.kIz));
    climberTable.putValue("Climber kFF", NetworkTableValue.makeDouble(ClimberConstants.kFF));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // climberTable.putValue("Climber Left Motor Raw Encoder",
    //     NetworkTableValue.makeDouble(climbMotorLeft.getEncoder().getPosition()));
    // climberTable.putValue("Climber Right Motor Raw Encoder",
    //     NetworkTableValue.makeDouble(climbMotorRight.getEncoder().getPosition()));
    climberTable.putValue("Climber Left Motor Encoder",
        NetworkTableValue.makeDouble(getPosition(climbMotorLeft)));
    climberTable.putValue("Climber Right Motor Encoder",
        NetworkTableValue.makeDouble(getPosition(climbMotorRight)));
    climberTable.putValue("Left at Bottom", NetworkTableValue.makeBoolean(atBottom(climbMotorLeft)));
    climberTable.putValue("Right at Bottom", NetworkTableValue.makeBoolean(atBottom(climbMotorRight)));
  }

  // public void climbTo(double output) {
  // climbController.setReference(output, CANSparkMax.ControlType.kPosition);
  // }

  public boolean atBottom(CANSparkMax motor) {
    // if (climbing) {
    if (motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()) {
      return true;
    } else {
      return false;
    }
    // }
    // return false;

    // if ((m_encoder.getVelocity() < Math.abs(0.1)) || (m_encoder.getVelocity() <
    // 0)) {
    // return true;
    // } else {
    // return false;
    // }
  }

  public void climb(double climbSpeed) {
    if (climbSpeed != 0) {
      climbing = true;
    } else {
      climbing = false;
    }
    climbMotorLeft.set(climbSpeed);
    climbMotorRight.set(climbSpeed);
  }

  public Command climbTo(double climbSpeed) {
    return new StartEndCommand(
        () -> {
          climbing = true;
          climbMotorLeft.set(climbSpeed);
          climbMotorRight.set(climbSpeed);
        },
        () -> {
          climbMotorLeft.set(0);
          climbMotorRight.set(0);
          climbing = false;
        });
  }

  public Command climbTo(CANSparkMax motor, double climbSpeed) {
    return new StartEndCommand(
        () -> {
          climbing = true;
          motor.set(climbSpeed);
        },
        () -> {
          motor.set(0);
          climbing = false;
        });
  }

  public Command calibrate() {
    return new ParallelCommandGroup(
        climbTo(climbMotorLeft, -ClimberConstants.calibrateSpeed).until(() -> {
          return atBottom(climbMotorLeft);
        }).andThen(new InstantCommand(() -> {
          resetCalibrationLeft();

          climbMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

          climbMotorLeft.burnFlash();
        })),
        climbTo(climbMotorRight, -ClimberConstants.calibrateSpeed).until(() -> {
          return atBottom(climbMotorRight);
        }).andThen(new InstantCommand(() -> {
          resetCalibrationRight();

          climbMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);

          climbMotorRight.burnFlash();
        })));
  }

  public void resetCalibrationLeft() {
    if (atBottom(climbMotorLeft)) {
      System.out.println("at bottom");
      m_leftOffset = ClimberConstants.bottomPosition - climbMotorLeft.getEncoder().getPosition();
      climbMotorLeft.setSoftLimit(SoftLimitDirection.kForward, (float) (ClimberConstants.topPosition - m_leftOffset));
      climbMotorLeft.setSoftLimit(SoftLimitDirection.kReverse,
          (float) (ClimberConstants.bottomPosition - m_leftOffset));

      // climbMotorLeft.getEncoder().setPosition(ClimberConstants.bottomPosition);

    }
  }

  public void resetCalibrationRight() {
    if (atBottom(climbMotorRight)) {
      System.out.println("at bottom");
      m_rightOffset = ClimberConstants.bottomPosition - climbMotorRight.getEncoder().getPosition();
      climbMotorRight.setSoftLimit(SoftLimitDirection.kForward, (float) (ClimberConstants.topPosition - m_rightOffset));
      climbMotorRight.setSoftLimit(SoftLimitDirection.kReverse,
          (float) (ClimberConstants.bottomPosition - m_rightOffset));
      // climbMotorRight.getEncoder().setPosition(ClimberConstants.bottomPosition);

    }
  }

  public void adjustPivot(PivotSubsystem m_pivotSubsystem) {
    if (getPosition(climbMotorLeft) <= ClimberConstants.adjustPivotThreshold
        || getPosition(climbMotorRight) <= ClimberConstants.adjustPivotThreshold) {
      double pivotAngle = ((getPosition(climbMotorLeft) - ClimberConstants.bottomPosition)
          * (PivotConstants.encoderAt90 - PivotConstants.climbAdjustmentPosition))
          / (ClimberConstants.adjustPivotThreshold - ClimberConstants.bottomPosition)
          + PivotConstants.climbAdjustmentPosition;
      m_pivotSubsystem.setPosition(pivotAngle);
    }
  }

  public double getPosition(CANSparkMax motor) {
    if (motor == climbMotorLeft) {
      return motor.getEncoder().getPosition() + m_leftOffset;
    }
    if (motor == climbMotorRight) {
      return motor.getEncoder().getPosition() + m_rightOffset;
    }
    return 0;
  }
}

// Adjust pivot:
// when the climber arm reaches a certain height/angle, rotate the pivot forward
// gradually