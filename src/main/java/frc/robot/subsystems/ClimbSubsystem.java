// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class ClimbSubsystem extends SubsystemBase {
  CANSparkMax climbMotor = new CANSparkMax(IDConstants.climbMotorID, MotorType.kBrushless);
  SparkPIDController climbController; 
  RelativeEncoder m_encoder;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable climberTable = inst.getTable("climber-table"); // TODO: Make these names consistant (I like the table one)
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    // climbMotor.getConfigurator().apply(config); // TODO: Is there a configuration for CANSparkMax?
    climbMotor.restoreFactoryDefaults();

    climbController.setP(ClimberConstants.kP);
    climbController.setI(ClimberConstants.kI);
    climbController.setD(ClimberConstants.kD);
    climbController.setIZone(ClimberConstants.kIz);
    climbController.setFF(ClimberConstants.kFF);
    climbController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    climbController.setReference(500.0, CANSparkMax.ControlType.kPosition);

    climbMotor.burnFlash();

    climberTable.putValue("Climber kP", NetworkTableValue.makeDouble(ClimberConstants.kP));
    climberTable.putValue("Climber kI", NetworkTableValue.makeDouble(ClimberConstants.kI));
    climberTable.putValue("Climber kD", NetworkTableValue.makeDouble(ClimberConstants.kD));
    climberTable.putValue("Climber kIz", NetworkTableValue.makeDouble(ClimberConstants.kIz));
    climberTable.putValue("Climber kFF", NetworkTableValue.makeDouble(ClimberConstants.kFF));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbTo(double output) {
    climbController.setReference(output, CANSparkMax.ControlType.kPosition);
  }

  public boolean atBottom() {
    if ((m_encoder.getVelocity() < Math.abs(0.1)) || (m_encoder.getVelocity() < 0)) {
      return true;
    }
    else {
      return false;
    }
  }
}