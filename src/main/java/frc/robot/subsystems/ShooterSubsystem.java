// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX m_frontMotor;
  private TalonFX m_backMotor;
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable shooterTable = inst.getTable("shooter-speed-PID");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // TODO: move to constants 
    m_frontMotor = new TalonFX(IDConstants.shooterMotor1ID);
    m_backMotor = new TalonFX(IDConstants.shooterMotor2ID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    m_frontMotor.getConfigurator().apply(config);
    m_backMotor.getConfigurator().apply(config);

    shooterTable.putValue("Shooter Speed 1", NetworkTableValue.makeDouble(ShooterConstants.shooterSpeed1));
    shooterTable.putValue("Shooter Speed 2", NetworkTableValue.makeDouble(ShooterConstants.shooterSpeed2));
    shooterTable.putValue("Shooter kV", NetworkTableValue.makeDouble(0));
    shooterTable.putValue("Shooter kP", NetworkTableValue.makeDouble(0));
    shooterTable.putValue("Shooter kI", NetworkTableValue.makeDouble(0));
    shooterTable.putValue("Shooter kD", NetworkTableValue.makeDouble(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterTable.putValue("frontMotor Velocity", NetworkTableValue.makeDouble(m_frontMotor.getVelocity().getValue()));
    shooterTable.putValue("backMotor Velocity", NetworkTableValue.makeDouble(m_backMotor.getVelocity().getValue()));

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kV = ShooterConstants.motor1kV;
    slot0Configs.kP = ShooterConstants.motor1kP;
    slot0Configs.kI = ShooterConstants.motor1kI;
    slot0Configs.kD = ShooterConstants.motor1kD;
    m_frontMotor.getConfigurator().apply(slot0Configs);
    m_backMotor.getConfigurator().apply(slot0Configs);
  }

  public void setShooter(double output1, double output2) {
    m_velocity.Slot = 0;
    m_frontMotor.setControl(m_velocity.withVelocity(-output1));
    m_backMotor.setControl(m_velocity.withVelocity(output2));
  }

  public boolean atSpeed() {
    if (Math.abs(m_frontMotor.getClosedLoopError().getValue()) < ShooterConstants.errorThreshold 
     && Math.abs(m_backMotor.getClosedLoopError().getValue()) < ShooterConstants.errorThreshold) {
      return true;
    }
    return false;
  }

  public void stopShooter() {
    m_frontMotor.setControl(new DutyCycleOut(0));
    m_backMotor.setControl(new DutyCycleOut(0));
  }
}
