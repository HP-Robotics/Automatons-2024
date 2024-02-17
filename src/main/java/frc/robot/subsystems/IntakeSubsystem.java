// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_motor = new TalonFX(IDConstants.intakeMotorID,"CANivore");
  CANSparkMax m_vanguardLeft = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax m_vanguardRight = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);

  public BeamBreak m_beambreak;

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
    m_vanguardRight.follow(m_vanguardLeft, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeTable.putValue("Beam Broken", NetworkTableValue.makeBoolean(m_beambreak.beamBroken()));
  }

  public void runIntake(double output, double vanguardOutput) {
    m_motor.setControl(new DutyCycleOut(output));
    m_vanguardLeft.set(vanguardOutput);
  };
}
