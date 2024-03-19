// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class ClimbSubsystem extends SubsystemBase {
  public CANSparkMax climbMotorLeft = new CANSparkMax(IDConstants.climbMotorLeftID, MotorType.kBrushless);
  public CANSparkMax climbMotorRight = new CANSparkMax(IDConstants.climbMotorRightID, MotorType.kBrushless); 
  SparkPIDController climbController; 
  RelativeEncoder m_encoder;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable climberTable = inst.getTable("climber-table"); // TODO: Make these names consistant (I like table)

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    // climbMotor.getConfigurator().apply(config); // TODO: Is there a configuration for CANSparkMax?
    climbMotorLeft.restoreFactoryDefaults();
    climbMotorRight.restoreFactoryDefaults();
   

    // climbController.setP(ClimberConstants.kP);
    // climbController.setI(ClimberConstants.kI);
    // climbController.setD(ClimberConstants.kD);
    // climbController.setIZone(ClimberConstants.kIz);
    // climbController.setFF(ClimberConstants.kFF);
    // climbController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    //climbController.setReference(500.0, CANSparkMax.ControlType.kPosition);

    // climbMotorRight.follow(climbMotorLeft,true);
    climbMotorRight.setInverted(true);
    climbMotorLeft.setSoftLimit(SoftLimitDirection.kForward, 130);  // JPW TODO - magic number
    climbMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);
    climbMotorRight.setSoftLimit(SoftLimitDirection.kForward, 130);  // JPW TODO - magic number
    climbMotorRight.setSoftLimit(SoftLimitDirection.kReverse, 0);

    climbMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
    climbMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climbMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    climbMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

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
    climberTable.putValue("Climber Left Motor Encoder", NetworkTableValue.makeDouble(climbMotorLeft.getEncoder().getPosition()));
    climberTable.putValue("Climber Right Motor Encoder", NetworkTableValue.makeDouble(climbMotorRight.getEncoder().getPosition()));
    
  }

  // public void climbTo(double output) {
  //   climbController.setReference(output, CANSparkMax.ControlType.kPosition);
  // }

  public boolean atBottom() {
    if ((m_encoder.getVelocity() < Math.abs(0.1)) || (m_encoder.getVelocity() < 0)) { // JPW TODO - magic number
      return true;
    } else {
      return false;
    }
  }
}