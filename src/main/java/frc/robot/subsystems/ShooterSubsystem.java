// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX m_leftMotor;
  private TalonFX m_rightMotor;
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable shooterTable = inst.getTable("shooter-subsystem");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftMotor = new TalonFX(IDConstants.leftMotorID, "CANivore");
    m_rightMotor = new TalonFX(IDConstants.rightMotorID, "CANivore");
    TalonFXConfiguration config = new TalonFXConfiguration();
    var rampConfigs = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ShooterConstants.rampTimeTo300s);
    var currentConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(ShooterConstants.currentLimit)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentThreshold(ShooterConstants.currentThreshold)
        .withSupplyTimeThreshold(ShooterConstants.currentTimeThreshold);

    m_leftMotor.getConfigurator().apply(config);
    m_rightMotor.getConfigurator().apply(config);

    m_leftMotor.getConfigurator().apply(rampConfigs);
    m_rightMotor.getConfigurator().apply(rampConfigs);
    m_leftMotor.getConfigurator().apply(currentConfigs);
    m_rightMotor.getConfigurator().apply(currentConfigs);

    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    shooterTable.putValue("leftMotor Setpoint", NetworkTableValue.makeDouble(ShooterConstants.shooterSpeedLeft));
    shooterTable.putValue("rightMotor Setpoint", NetworkTableValue.makeDouble(ShooterConstants.shooterSpeedRight));
    shooterTable.putValue("Shooter kV", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskV));
    shooterTable.putValue("Shooter kP", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskP));
    shooterTable.putValue("Shooter kI", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskI));
    shooterTable.putValue("Shooter kD", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskD));

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kV = ShooterConstants.shooterMotorskV;
    slot0Configs.kP = ShooterConstants.shooterMotorskP;
    slot0Configs.kI = ShooterConstants.shooterMotorskI;
    slot0Configs.kD = ShooterConstants.shooterMotorskD;
    m_leftMotor.getConfigurator().apply(slot0Configs);
    m_rightMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterTable.putValue("leftMotor Velocity", NetworkTableValue.makeDouble(m_leftMotor.getVelocity().getValue()));
    shooterTable.putValue("rightMotor Velocity", NetworkTableValue.makeDouble(m_rightMotor.getVelocity().getValue()));
    shooterTable.putValue("Shooter At Speed", NetworkTableValue.makeBoolean(this.atSpeed()));

    // Slot0Configs slot0Configs = new Slot0Configs();
    // slot0Configs.kV = ShooterConstants.shooterMotorskV;
    // slot0Configs.kP = ShooterConstants.shooterMotorskP;
    // slot0Configs.kI = ShooterConstants.shooterMotorskI;
    // slot0Configs.kD = ShooterConstants.shooterMotorskD;
    // m_leftMotor.getConfigurator().apply(slot0Configs);
    // m_rightMotor.getConfigurator().apply(slot0Configs);

  }

  public void setShooter(double output1, double output2) {
    m_velocity.Slot = 0;
    m_leftMotor.setControl(m_velocity.withVelocity(output1));
    m_rightMotor.setControl(m_velocity.withVelocity(output2));
    // m_leftMotor.setControl(new DutyCycleOut(output1));
    // m_rightMotor.setControl(new DutyCycleOut(output2));
  }

  public boolean atSpeed() {
    // return true;
    if (Math.abs(m_leftMotor.getClosedLoopError().getValue()) < ShooterConstants.errorThreshold
        && Math.abs(m_rightMotor.getClosedLoopError().getValue()) < ShooterConstants.errorThreshold) {
      return true;
    }
    return false;
  }

  public void stopShooter() {
    m_leftMotor.setControl(new DutyCycleOut(0));
    m_rightMotor.setControl(new DutyCycleOut(0));
  }
}
