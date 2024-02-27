// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SnuffilatorConstants;

public class SnuffilatorSubsystem extends SubsystemBase {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable snuffyTable = inst.getTable("snuffy-table");
  CANSparkMax m_snuffilator = new CANSparkMax(IDConstants.snuffilatorID, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new SnuffilatorSubsystem. */
  public SnuffilatorSubsystem() {
    m_snuffilator.restoreFactoryDefaults();
    m_snuffilator.setSmartCurrentLimit(SnuffilatorConstants.stallCurrentLimit, SnuffilatorConstants.freeCurrentLimit);
    m_snuffilator.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double output) {
    m_snuffilator.set(output);
  }
}
