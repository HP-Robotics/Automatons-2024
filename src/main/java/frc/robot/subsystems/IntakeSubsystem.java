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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
  public TalonFX m_motor = new TalonFX(IDConstants.intakeMotorID,"CANivore");
  CANSparkMax m_vanguardLeft = new CANSparkMax(IDConstants.vanguardLeftID, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax m_vanguardRight = new CANSparkMax(IDConstants.vanguardRightID, CANSparkLowLevel.MotorType.kBrushless);

  public BeamBreak m_beambreak;
  public boolean intakeOn = false; //intake button is pressed
  public boolean intakeFire = false; //fire button is pressed
  public boolean intakeYuck = false; //yuck button is pressed
  public boolean beambreakState = false; //trigger beambreak sees note & intakeOn

  public double m_lastOutput = 0.0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = inst.getTable("intake-table");


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    m_motor.getConfigurator().apply(config);

    intakeTable.getEntry("Intake Setpoint").getDouble(IntakeConstants.intakeSpeed);

    m_beambreak = new BeamBreak(PortConstants.IntakeBeamBreak);

    m_vanguardLeft.restoreFactoryDefaults();
    m_vanguardRight.restoreFactoryDefaults();
    m_vanguardLeft.setSmartCurrentLimit(10);
    m_vanguardRight.setSmartCurrentLimit(10);
    m_vanguardRight.follow(m_vanguardLeft, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeTable.putValue("Beam Broken", NetworkTableValue.makeBoolean(m_beambreak.beamBroken()));

    intakeTable.putValue("Intake On", NetworkTableValue.makeBoolean(intakeOn));
    intakeTable.putValue("Intake Fire", NetworkTableValue.makeBoolean(intakeFire));
    intakeTable.putValue("Intake Yuck", NetworkTableValue.makeBoolean(intakeYuck));
    intakeTable.putValue("Intake BeamBreak", NetworkTableValue.makeBoolean(beambreakState));
  }

  public void runIntake(double output, double vanguardOutput) {
    if (output != m_lastOutput) {
      if (output == 0) {
        m_motor.setControl(new NeutralOut());
      }
      else {
        m_motor.setControl(new DutyCycleOut(output));
      }
      m_vanguardLeft.set(vanguardOutput);
    }
    m_lastOutput = output;
  };

  public void intakeButtonPressed () {
    if (!intakeFire && !intakeYuck && !beambreakState) {
      intakeOn = true;
    }
  }
  
  public void intakeButtonReleased () {
    if (intakeOn) {
      intakeOn = false;
    }
  }

  public void yuckButtonPressed () {
    if (!intakeFire) {
      intakeYuck = true;
    }
  }

  public void yuckButtonReleased () {
    intakeYuck = false;
  }

  public void fireButtonPressed () {
    if (!intakeYuck) {
      intakeFire = true;
    }
  }

  public void fireButtonReleased () {
    intakeFire = false;
  }

  public void runOnlyVanguard (double output) {
    m_vanguardLeft.set(output); // TODO: Add right
  }

}
