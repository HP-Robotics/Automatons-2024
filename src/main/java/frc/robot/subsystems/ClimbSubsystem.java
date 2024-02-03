// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;

public class ClimbSubsystem extends SubsystemBase {
  TalonFX m_motor4 = new TalonFX(ClimbConstants.climbMotorID);
  /** Creates a new IntakeSubsystem. */
  public ClimbSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    m_motor4.getConfigurator().apply(config);

    SmartDashboard.putNumber("Intake Speed", IntakeConstants.intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motor4.set(climb_speed, 0.4);
  }

  public void climb() {
    m_motor4.setControl(new DutyCycleOut(climb_speed));
  }
}
