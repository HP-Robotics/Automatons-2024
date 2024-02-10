package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PortConstants;
import edu.wpi.first.math.controller.PIDController;
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

	/** Creates a new ExampleSubsystem. */
	public PivotSubsystem() {
		m_motorR = new TalonFX(IDConstants.rightPivotID);
		m_motorL = new TalonFX(IDConstants.leftPivotID);
		Slot0Configs motorConfigs = new Slot0Configs();
		m_motorL.getConfigurator().apply(new TalonFXConfiguration());
		m_motorR.getConfigurator().apply(new TalonFXConfiguration());
		m_motorR.setNeutralMode(NeutralModeValue.Brake);
		m_motorL.setNeutralMode(NeutralModeValue.Brake);

		motorConfigs.kP = SmartDashboard.getNumber("kP", PivotConstants.kP); //won't send to falcon
		motorConfigs.kI = SmartDashboard.getNumber("kI", PivotConstants.kI);
		motorConfigs.kD = SmartDashboard.getNumber("kD", PivotConstants.kD);

		m_motorL.setControl(new Follower(IDConstants.rightPivotID, true));
		
		m_absEncoder = new DutyCycleEncoder(PortConstants.pivotAbsEncoderID);
		m_pivotController = new PIDController(motorConfigs.kP, motorConfigs.kI, motorConfigs.kD);
		m_pivotController.setSetpoint(m_absEncoder.getAbsolutePosition());
	}

	public void periodic() {
		pivotTable.putValue("Absolute Encoder Position",
				NetworkTableValue.makeDouble(m_absEncoder.getAbsolutePosition()));
		if(m_absEncoder.getAbsolutePosition() != 0) {
			double output = m_pivotController.calculate(m_absEncoder.getAbsolutePosition());
			if(m_usePID) {
				m_motorR.setControl(new DutyCycleOut(output));
			}
			m_absoluteBroken = false;
		} else {
			if(!m_absoluteBroken) {
				m_motorR.setControl(new DutyCycleOut(0));
				m_absoluteBroken = true;
			}
		}
		pivotTable.putValue("Pivot Power",NetworkTableValue.makeDouble(m_motorR.getDutyCycle().getValueAsDouble()));
		pivotTable.putValue("Pivot Position",NetworkTableValue.makeDouble(m_motorR.getPosition().getValueAsDouble()));

		if(m_pivotController.getP() != pivotTable.getEntry("kP").getDouble(m_pivotController.getP())) {
			m_pivotController.setP(pivotTable.getEntry("kP").getDouble(m_pivotController.getP()));
		}
		if(m_pivotController.getI() != pivotTable.getEntry("kI").getDouble(m_pivotController.getI())) {
			m_pivotController.setI(pivotTable.getEntry("kI").getDouble(m_pivotController.getI()));
		}
		if(m_pivotController.getD() != pivotTable.getEntry("kD").getDouble(m_pivotController.getD())) {
			m_pivotController.setD(pivotTable.getEntry("kD").getDouble(m_pivotController.getD()));
		}
	}

	public void togglePID() {
		m_usePID = !m_usePID;
		if(m_usePID) {
			m_pivotController.setSetpoint(m_absEncoder.getAbsolutePosition());
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
	};

	public void setPosition(double position) {
		m_pivotController.setSetpoint(position);
	};
}