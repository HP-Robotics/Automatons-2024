// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_motor3;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFX m_motor3 = new TalonFX(3);

    TalonFXConfiguration config = new TalonFXConfiguration();
    m_motor3.getConfigurator().apply(config);

    SmartDashboard.putNumber("Intake Speed", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void useIntake(double output) {
    m_motor3.setControl(new DutyCycleOut(output));
  };
}
