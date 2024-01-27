// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX m_motor1;
  private TalonFX m_motor2;
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // TODO: move to constants
    m_motor1 = new TalonFX(ShooterConstants.motor1ID);
    m_motor2 = new TalonFX(ShooterConstants.motor2ID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    m_motor1.getConfigurator().apply(config);
    m_motor2.getConfigurator().apply(config);

    SmartDashboard.putNumber("Shooter Speed 1", ShooterConstants.shooterSpeed1);
    SmartDashboard.putNumber("Shooter Speed 2", ShooterConstants.shooterSpeed2);
    SmartDashboard.putNumber("Shooter kV", 0);
    SmartDashboard.putNumber("Shooter kP", 0);
    SmartDashboard.putNumber("Shooter kI", 0);
    SmartDashboard.putNumber("Shooter kD", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("motor1 Velocity", m_motor1.getVelocity().getValue());
    SmartDashboard.putNumber("motor2 Velocity", m_motor2.getVelocity().getValue());
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kV = ShooterConstants.motor1kV;
    slot0Configs.kP = ShooterConstants.motor1kP;
    slot0Configs.kI = ShooterConstants.motor1kI;
    slot0Configs.kD = ShooterConstants.motor1kD;
    m_motor1.getConfigurator().apply(slot0Configs);
    m_motor2.getConfigurator().apply(slot0Configs);
  }

  public void setShooter(double output1, double output2) {
    m_velocity.Slot = 0;
    m_motor1.setControl(m_velocity.withVelocity(-output1));
    m_motor2.setControl(m_velocity.withVelocity(output2));
  }

  public void stopShooter() {
    m_motor1.setControl(new DutyCycleOut(0));
    m_motor2.setControl(new DutyCycleOut(0));
  }
}
