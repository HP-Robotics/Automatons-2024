// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;

public class IntakeSubsystem extends SubsystemBase {
  public TalonFX m_motor = new TalonFX(IDConstants.intakeMotorID, "CANivore");
  CANSparkMax m_vanguardSide = new CANSparkMax(IDConstants.vanguardSideID, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax m_vanguardFront = new CANSparkMax(IDConstants.vanguardFrontID, CANSparkLowLevel.MotorType.kBrushless);

  public BeamBreak m_beambreak;
  public boolean m_isIntaking = false; // intake button is pressed
  public boolean m_isFiring = false; // fire button is pressed
  public boolean m_isYucking = false; // yuck button is pressed
  public boolean m_isLoaded = false; // trigger beambreak sees note & intakeOn

  public double m_lastOutput = 0.0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = inst.getTable("intake-table");

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    m_motor.getConfigurator().apply(config);

    intakeTable.getEntry("Intake Setpoint").getDouble(IntakeConstants.intakeSpeed);

    m_beambreak = new BeamBreak(PortConstants.IntakeBeamBreak);

    m_vanguardSide.restoreFactoryDefaults();
    m_vanguardFront.restoreFactoryDefaults();
    m_vanguardSide.setSmartCurrentLimit(IntakeConstants.vanguardCurrentLimitSide);
    m_vanguardSide.setInverted(true);
    m_vanguardFront.setSmartCurrentLimit(IntakeConstants.vanguardCurrentLimitFront);

    m_vanguardSide.burnFlash();
    m_vanguardFront.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeTable.putValue("Beam Broken", NetworkTableValue.makeBoolean(m_beambreak.beamBroken()));
    intakeTable.putValue("Intake On", NetworkTableValue.makeBoolean(m_isIntaking));
    intakeTable.putValue("Intake Fire", NetworkTableValue.makeBoolean(m_isFiring));
    intakeTable.putValue("Intake Yuck", NetworkTableValue.makeBoolean(m_isYucking));
    intakeTable.putValue("Intake BeamBreak", NetworkTableValue.makeBoolean(m_isLoaded));
  }

  public void runIntake(double output, double vanguardOutputSide, double vangaurdOutputFront) {
    if (output != m_lastOutput) {
      if (output == 0) {
        m_motor.setControl(new NeutralOut());
      } else {
        m_motor.setControl(new DutyCycleOut(output));
      }
      // m_vanguardSide.set(vanguardOutputSide);
      m_vanguardFront.set(vangaurdOutputFront);
    }
    m_lastOutput = output;
  };

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

  public void runOnlyVanguard(double vangaurdOutputFront, double vanguardOutputSide) {
    // m_vanguardSide.set(vanguardOutputSide);
    m_vanguardFront.set(vangaurdOutputFront);

  }
}