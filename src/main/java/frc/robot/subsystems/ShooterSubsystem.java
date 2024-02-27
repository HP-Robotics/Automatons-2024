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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX m_frontMotor;
  private TalonFX m_backMotor;
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);
  private final CommandJoystick m_rumbleJoystick;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable shooterTable = inst.getTable("shooter-subsystem");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CommandJoystick joystick) {
    m_frontMotor = new TalonFX(IDConstants.frontMotorID, "CANivore");
    m_backMotor = new TalonFX(IDConstants.backMotorID, "CANivore");
    TalonFXConfiguration config = new TalonFXConfiguration();
    m_rumbleJoystick = joystick;
    var rampConfigs = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ShooterConstants.rampTimeTo300s);
    var currentConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(ShooterConstants.currentLimit)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentThreshold(ShooterConstants.currentThreshold)
        .withSupplyTimeThreshold(ShooterConstants.currentTimeThreshold); // TODO: This isn't working we don't know why

    m_frontMotor.getConfigurator().apply(config);
    m_backMotor.getConfigurator().apply(config);

    m_frontMotor.getConfigurator().apply(rampConfigs);
    m_backMotor.getConfigurator().apply(rampConfigs);
    m_frontMotor.getConfigurator().apply(currentConfigs);
    m_backMotor.getConfigurator().apply(currentConfigs);

    m_frontMotor.setInverted(true);

    shooterTable.putValue("frontMotor Setpoint", NetworkTableValue.makeDouble(ShooterConstants.shooterSpeedFront));
    shooterTable.putValue("backMotor Setpoint", NetworkTableValue.makeDouble(ShooterConstants.shooterSpeedBack));
    shooterTable.putValue("Shooter kV", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskV));
    shooterTable.putValue("Shooter kP", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskP));
    shooterTable.putValue("Shooter kI", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskI));
    shooterTable.putValue("Shooter kD", NetworkTableValue.makeDouble(ShooterConstants.shooterMotorskD));

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kV = ShooterConstants.shooterMotorskV;
    slot0Configs.kP = ShooterConstants.shooterMotorskP;
    slot0Configs.kI = ShooterConstants.shooterMotorskI;
    slot0Configs.kD = ShooterConstants.shooterMotorskD;
    m_frontMotor.getConfigurator().apply(slot0Configs);
    m_backMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterTable.putValue("frontMotor Velocity", NetworkTableValue.makeDouble(m_frontMotor.getVelocity().getValue()));
    shooterTable.putValue("backMotor Velocity", NetworkTableValue.makeDouble(m_backMotor.getVelocity().getValue()));
    shooterTable.putValue("Shooter At Speed", NetworkTableValue.makeBoolean(this.atSpeed()));

    // Slot0Configs slot0Configs = new Slot0Configs();
    // slot0Configs.kV = ShooterConstants.shooterMotorskV;
    // slot0Configs.kP = ShooterConstants.shooterMotorskP;
    // slot0Configs.kI = ShooterConstants.shooterMotorskI;
    // slot0Configs.kD = ShooterConstants.shooterMotorskD;
    // m_frontMotor.getConfigurator().apply(slot0Configs);
    // m_backMotor.getConfigurator().apply(slot0Configs);

  }

  public void setShooter(double output1, double output2) {
    m_velocity.Slot = 0;
    m_frontMotor.setControl(m_velocity.withVelocity(output1));
    m_backMotor.setControl(m_velocity.withVelocity(output2));
    // m_frontMotor.setControl(new DutyCycleOut(output1));
    // m_backMotor.setControl(new DutyCycleOut(output2));

    if (output1 > 0.1 || output2 > 0.1) {
      m_rumbleJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      m_rumbleJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  public boolean atSpeed() {
    // return true;
    if (Math.abs(m_frontMotor.getClosedLoopError().getValue()) < ShooterConstants.errorThreshold
        && Math.abs(m_backMotor.getClosedLoopError().getValue()) < ShooterConstants.errorThreshold) {
      return true;
    }
    return false;
  }

  public void stopShooter() {
    m_frontMotor.setControl(new DutyCycleOut(0));
    m_backMotor.setControl(new DutyCycleOut(0));
    m_rumbleJoystick.getHID().setRumble(RumbleType.kBothRumble, 0);
  }
}
