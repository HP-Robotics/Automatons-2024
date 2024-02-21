package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  private TalonFX m_motorR;
  private TalonFX m_motorL;
  private DutyCycleEncoder m_absEncoder;
  private PIDController m_pivotController;
  private boolean m_usePID = PivotConstants.startWithPID;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable pivotTable = inst.getTable("pivot-table");
  private boolean m_absoluteBroken = false;
  private ArmFeedforward m_armGraivty;
  private LinearFilter m_filter;

  /** Creates a new ExampleSubsystem. */
  public PivotSubsystem() {
    m_motorR = new TalonFX(IDConstants.rightPivotID, "CANivore");
    m_motorL = new TalonFX(IDConstants.leftPivotID, "CANivore");
    Slot0Configs motorConfigs = new Slot0Configs();
    m_motorL.getConfigurator().apply(new TalonFXConfiguration());
    m_motorR.getConfigurator().apply(new TalonFXConfiguration());
    m_motorR.setNeutralMode(NeutralModeValue.Brake);
    m_motorR.setInverted(true);
    m_motorL.setNeutralMode(NeutralModeValue.Brake);
    var rampConfigs = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(PivotConstants.rampTimeTo300s);
    var currentConfigs = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(PivotConstants.currentLimit)
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentThreshold(PivotConstants.currentThreshold)
      .withSupplyTimeThreshold(PivotConstants.currentTimeThreshold); // TODO: This isn't working we don't know why

    pivotTable.putValue("kP", NetworkTableValue.makeDouble(PivotConstants.kP));
    pivotTable.putValue("kI", NetworkTableValue.makeDouble(PivotConstants.kI));
    pivotTable.putValue("kD", NetworkTableValue.makeDouble(PivotConstants.kD));
    pivotTable.putValue("kG", NetworkTableValue.makeDouble(PivotConstants.kG));

    motorConfigs.kP = pivotTable.getEntry("kP").getDouble(PivotConstants.kP); // we won't send to falcon
    motorConfigs.kI = pivotTable.getEntry("kI").getDouble(PivotConstants.kI);
    motorConfigs.kD = pivotTable.getEntry("kD").getDouble(PivotConstants.kD);

    m_motorL.setControl(new Follower(IDConstants.rightPivotID, true));

    m_absEncoder = new DutyCycleEncoder(PortConstants.pivotAbsEncoderID);
    m_pivotController = new PIDController(motorConfigs.kP, motorConfigs.kI, motorConfigs.kD);
    m_pivotController.setSetpoint(0.4);
    m_pivotController.setTolerance(0.015);
    m_pivotController.setIZone(0.014);

    m_motorR.getConfigurator().apply(rampConfigs);
    m_motorR.getConfigurator().apply(currentConfigs);

    m_motorL.getConfigurator().apply(rampConfigs);
    m_motorL.getConfigurator().apply(currentConfigs);

    m_armGraivty = new ArmFeedforward(0, PivotConstants.kG, 0);
    m_filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  }

  public void periodic() {
    pivotTable.putValue("Absolute Encoder Position",
        NetworkTableValue.makeDouble(m_absEncoder.getAbsolutePosition()));
    double grav = -m_armGraivty.calculate(encoderToRadians(m_pivotController.getSetpoint()), 0);
    if (m_absEncoder.getAbsolutePosition() != 0 && m_absEncoder.getAbsolutePosition() != 1) {
      double filtered_Encoder = m_filter.calculate(m_absEncoder.getAbsolutePosition());
      double output = m_pivotController.calculate(filtered_Encoder);
      output = output + grav;
      if (m_usePID) {
        if (m_pivotController.atSetpoint()) {
          m_motorR.setControl(new DutyCycleOut(output));
        } else {
          m_motorR.setControl(new DutyCycleOut(output));
        }
      }
      m_absoluteBroken = false;
      // pivotTable.putValue("Grav Propotion", NetworkTableValue.makeDouble(grav));
      // pivotTable.putValue("Commanded Output", NetworkTableValue.makeDouble(output));
      // pivotTable.putValue("Filtered Input", NetworkTableValue.makeDouble(filtered_Encoder));

    } else {
      if (!m_absoluteBroken) {
        // m_motorR.setControl(new DutyCycleOut(grav));
        m_absoluteBroken = true;
      }
    }
    // pivotTable.putValue("Pivot
    // Power",NetworkTableValue.makeDouble(m_motorR.getDutyCycle().getValueAsDouble()));
    // pivotTable.putValue("Pivot
    // Position",NetworkTableValue.makeDouble(m_motorR.getPosition().getValueAsDouble()));
    // pivotTable.putValue("P Proportion",
    //     NetworkTableValue.makeDouble(m_pivotController.getPositionError() * m_pivotController.getP()));
    // pivotTable.putValue("D Proportion",
    //     NetworkTableValue.makeDouble(m_pivotController.getVelocityError() * m_pivotController.getD()));
    pivotTable.putValue("Pivot Setpoint", NetworkTableValue.makeDouble(m_pivotController.getSetpoint()));
    // pivotTable.putValue("Pivot
    // Error",NetworkTableValue.makeDouble(m_pivotController.getPositionError()));
    // pivotTable.putValue("Pivot
    // Angle",NetworkTableValue.makeDouble(Math.toDegrees(encoderToRadians(m_absEncoder.getAbsolutePosition()))));

    if (m_pivotController.getP() != pivotTable.getEntry("kP").getDouble(m_pivotController.getP())) {
      m_pivotController.setP(pivotTable.getEntry("kP").getDouble(m_pivotController.getP()));
    }
    if (m_pivotController.getI() != pivotTable.getEntry("kI").getDouble(m_pivotController.getI())) {
      m_pivotController.setI(pivotTable.getEntry("kI").getDouble(m_pivotController.getI()));
    }
    if (m_pivotController.getD() != pivotTable.getEntry("kD").getDouble(m_pivotController.getD())) {
      m_pivotController.setD(pivotTable.getEntry("kD").getDouble(m_pivotController.getD()));
    }
    if (m_armGraivty.kg != pivotTable.getEntry("kG").getDouble(m_armGraivty.kg)) {
      m_armGraivty = new ArmFeedforward(0, pivotTable.getEntry("kG").getDouble(m_pivotController.getD()), 0);
    }
  }

  public double encoderToRadians(double encoder) {
    return (encoder + 0.25 - PivotConstants.encoderAt90) * 2 * Math.PI; // 0.25 is for 90 degree offset
  }

  public void togglePID() {
    m_usePID = !m_usePID;
    if (m_usePID) {
      m_pivotController.setSetpoint(m_absEncoder.getAbsolutePosition());// TODO constrain setpoint to within limit
                                                                        // switches
    }
  }

  public boolean getUsePID() {
    return m_usePID;
  }

  public double getCurrentPosition() {
    return m_absEncoder.getAbsolutePosition();
  }

  public void setSpeed(double output) {
    m_motorR.setControl(new DutyCycleOut(output));
  }

  public double getMagicAngle(double distance) {
    return PivotConstants.magicConstants[0] * distance * distance + PivotConstants.magicConstants[1] * distance
        + PivotConstants.magicConstants[2];
  }

  public void setPosition(double position) { // TODO remove bad inputs
    m_pivotController.setSetpoint(position); // TODO constrain setpoint to within limit switches--make setpoint safe
                                             // method
    // System.out.println(position);
  };
  // TODO angle to sucesful shot, amp, speaker, and podium setpoint

  public boolean atPosition() {
    return m_pivotController.atSetpoint();
  }
}