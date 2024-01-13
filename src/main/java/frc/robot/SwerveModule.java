// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeoutException;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {

  public final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final double m_desired;
  private Double m_turningOffset;
  public final DutyCycleEncoder m_absEncoder;
  String m_name;
  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable driveTrainTable = inst.getTable("drive-train");

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   CAN ID for the drive motor.
   * @param turningMotorChannel CAN ID for the turning motor.
   * 
   * 
   * 
   */

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel, int absEncoder, double desired, String name) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = DriveConstants.drivekF;
    slot0Configs.kP = DriveConstants.drivekP;
    slot0Configs.kI = DriveConstants.drivekI;
    slot0Configs.kD = DriveConstants.drivekD;
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_driveMotor.getConfigurator().apply(slot0Configs);
    

    
    //m_driveMotor.configAllowableClosedloopError(0, DriveConstants.drivekAllowableError);

    m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
    m_turningMotor = new TalonFX(turningMotorChannel);
    var turningConfig = new Slot0Configs();
    turningConfig.kP = DriveConstants.turningkP;
    turningConfig.kI = DriveConstants.turningkI;
    turningConfig.kD = DriveConstants.turningkD;
    m_turningMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_turningMotor.getConfigurator().apply(turningConfig); 
    m_turningMotor.setNeutralMode(NeutralModeValue.Brake);
    // var turningFeedbackConfig = new FeedbackConfigs();
    // // turningFeedbackConfig.SensorToMechanismRatio = DriveConstants.turningGearRatio;
    // // m_turningMotor.getConfigurator().apply(turningFeedbackConfig);


    m_turningMotor.setInverted(true);
    m_absEncoder = new DutyCycleEncoder(absEncoder);
    m_turningOffset = 0.0;
    m_desired = desired;
    m_name = name;

  }

  public void resetOffset() {

    if (m_absEncoder.getAbsolutePosition() != 0) {
        m_turningMotor.getConfigurator().setPosition(0);
        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
    System.out.println(m_turningMotor.getPosition().getValueAsDouble());

      double offset = m_desired - m_absEncoder.getAbsolutePosition();
      m_turningOffset = offset * (DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio);
    }

  }

  public double radiansToTicks(double radians) {
    // drive ratio: 6.75:1
    // rotation ratio: 15.429:1
    return radians * ((DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio) / (2 * Math.PI));
  }

  public double ticksToRadians(double ticks) {
    return ticks * ((2 * Math.PI) / (DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio));
  }

  public double ticksToMeters(double ticks) {
    return ((ticks / DriveConstants.kEncoderResolution) * (2 * Math.PI * DriveConstants.kWheelRadius))
        / DriveConstants.driveGearRatio;
  }

  public double metersToTicks(double meters) {
    return (meters / (2 * Math.PI * DriveConstants.kWheelRadius)) * DriveConstants.kEncoderResolution
        * DriveConstants.driveGearRatio;

  }

  public void PositionControlWithAllowableClosedloopError(TalonFX motor, double position, double error) {
    // if (Math.abs(position - motor.getRotorPosition().getValueAsDouble())>error) {
      motor.setControl(new PositionDutyCycle(position));
    // }
    // else {
    //   motor.setControl(new NeutralOut());
    // }
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    driveTrainTable.putValue(m_name+ "turningCurrentSetpoint", NetworkTableValue.makeDouble(m_turningMotor.getClosedLoopReference().getValue()));
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(ticksToRadians(applyOffset(m_turningMotor.getRotorPosition().getValue()))));
    driveTrainTable.putValue(m_name + "turningAngle", NetworkTableValue.makeDouble(state.angle.getDegrees()));
    driveTrainTable.putValue(m_name + "turningOffset", NetworkTableValue.makeDouble(m_turningOffset));

    PositionControlWithAllowableClosedloopError(m_turningMotor, radiansToTicks(state.angle.getRadians()) + m_turningOffset, DriveConstants.turningkAllowableError);
    driveTrainTable.putValue(m_name + "turningSetpoint",NetworkTableValue.makeDouble(radiansToTicks(state.angle.getRadians()) + m_turningOffset));
    // m_driveMotor.setControl(new VelocityDutyCycle(metersToTicks(state.speedMetersPerSecond) / 10)); // the 10 is real, it turns ticks per second into ticks per 100ms

  }

  public void resetEncoderPosition(double desired, double current) {
    PositionControlWithAllowableClosedloopError(m_turningMotor,applyOffset(m_turningMotor.getRotorPosition().getValue())
        + DriveConstants.kEncoderResolution * DriveConstants.turningGearRatio * (desired - current),DriveConstants.turningkAllowableError);
  }


  public double applyOffset(double position) {
    return position - m_turningOffset;
  }

  public void resetTurningMotor() {
    m_turningMotor.setControl(new DutyCycleOut( 0));
    m_turningMotor.getConfigurator().setPosition(0);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        ticksToMeters(m_driveMotor.getRotorPosition().getValue()),
        new Rotation2d(ticksToRadians(applyOffset(m_turningMotor.getRotorPosition().getValue()))));
  }

  public double getDistance() {
    return m_driveMotor.getRotorPosition().getValue();
  }

  public double getEncoderAngle() {
    return m_turningMotor.getRotorPosition().getValue();
  }

  public double turnPower() {
    return m_turningMotor.get();
  }

  public double drivePower() {
    return m_driveMotor.get();
  }

  public void getDrivePower(String key) {
    SmartDashboard.putNumber(key + " Setpoint",
        m_driveMotor.getRotorVelocity().getValueAsDouble() - m_driveMotor.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(key + " Velocity", m_driveMotor.getRotorVelocity().getValueAsDouble());
  }

}
