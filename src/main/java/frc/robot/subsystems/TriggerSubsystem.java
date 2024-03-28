// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TriggerConstants;

public class TriggerSubsystem extends SubsystemBase {
  public TalonFX m_triggerMotor;
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);
  public BeamBreak m_beamBreak;
  public int beambreakCount = 0; // TODO remove for shooter v2?
  public double m_lastOutput = 0.0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable triggerTable = inst.getTable("trigger-subsystem");

  public boolean m_isIntaking = false; // intake button is pressed
  public boolean m_isFiring = false; // fire button is pressed
  public boolean m_isYucking = false; // yuck button is pressed
  public boolean m_isLoaded = false; // trigger beambreak sees note & intakeOn

  /** Creates a new ShooterSubsystem. */
  public TriggerSubsystem() {
    m_triggerMotor = new TalonFX(IDConstants.triggerMotorID, "CANivore");
    m_beamBreak = new BeamBreak(PortConstants.TriggerBeamBreak);

    TalonFXConfiguration config = new TalonFXConfiguration();

    m_triggerMotor.getConfigurator().apply(config);
    m_triggerMotor.setNeutralMode(NeutralModeValue.Brake);

    triggerTable.putValue("Trigger Setpoint", NetworkTableValue.makeDouble(TriggerConstants.triggerSpeed));
    triggerTable.putValue("Trigger kV", NetworkTableValue.makeDouble(0));
    triggerTable.putValue("Trigger kP", NetworkTableValue.makeDouble(0));
    triggerTable.putValue("Trigger kI", NetworkTableValue.makeDouble(0));
    triggerTable.putValue("Trigger kD", NetworkTableValue.makeDouble(0));

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kV = TriggerConstants.triggerkV;
    slot0Configs.kP = TriggerConstants.triggerkP;
    slot0Configs.kI = TriggerConstants.triggerkI;
    slot0Configs.kD = TriggerConstants.triggerkD;
    m_triggerMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    triggerTable.putValue("triggerMotor Velocity",
        NetworkTableValue.makeDouble(m_triggerMotor.getVelocity().getValue()));
    triggerTable.putValue("Beam Broken", NetworkTableValue.makeBoolean(m_beamBreak.beamBroken()));

    triggerTable.putValue("Trigger On", NetworkTableValue.makeBoolean(m_isIntaking));
    triggerTable.putValue("Trigger Fire", NetworkTableValue.makeBoolean(m_isFiring));
    triggerTable.putValue("Trigger Yuck", NetworkTableValue.makeBoolean(m_isYucking));
    triggerTable.putValue("Trigger BeamBreak", NetworkTableValue.makeBoolean(m_isLoaded));

  }

  public void setTrigger(double output) {
    m_velocity.Slot = 0;
    if (output != m_lastOutput) {
      m_triggerMotor.setControl(new DutyCycleOut(output));
    }
    m_lastOutput = output;
  }

  public void stopTrigger() {
    m_triggerMotor.setControl(new DutyCycleOut(0));
  }

  public void intakeButtonPressed() {
    if (!m_isFiring && !m_isYucking && !m_isLoaded) {
      m_isIntaking = true;
    }
  }

  public void intakeButtonReleased() {
    if (m_isIntaking) {
      m_isIntaking = false;
    }
  }

  public void yuckButtonPressed() {
    if (!m_isFiring) {
      m_isYucking = true;
    }
  }

  public void yuckButtonReleased() {
    m_isYucking = false;
  }

  public void fireButtonPressed() {
    if (!m_isYucking) {
      m_isFiring = true;
    }
  }

  public void fireButtonReleased() {
    m_isFiring = false;
  }

}
