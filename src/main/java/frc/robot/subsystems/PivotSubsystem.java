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
import frc.robot.Constants.ShooterConstants;
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
		m_motorR = new TalonFX(IDConstants.rightPivotID,"CANivore");
		m_motorL = new TalonFX(IDConstants.leftPivotID,"CANivore");
		Slot0Configs motorConfigs = new Slot0Configs();
		m_motorL.getConfigurator().apply(new TalonFXConfiguration());
		m_motorR.getConfigurator().apply(new TalonFXConfiguration());
		m_motorR.setNeutralMode(NeutralModeValue.Brake);
		m_motorR.setInverted(true);
		m_motorL.setNeutralMode(NeutralModeValue.Brake);

		pivotTable.putValue("kP", NetworkTableValue.makeDouble(PivotConstants.kP));
    	pivotTable.putValue("kI", NetworkTableValue.makeDouble(PivotConstants.kI));
    	pivotTable.putValue("kD", NetworkTableValue.makeDouble(PivotConstants.kD));

		motorConfigs.kP = pivotTable.getEntry("kP").getDouble(PivotConstants.kP); //we won't send to falcon
		motorConfigs.kI = pivotTable.getEntry("kI").getDouble(PivotConstants.kI);
		motorConfigs.kD = pivotTable.getEntry("kD").getDouble(PivotConstants.kD);

		m_motorL.setControl(new Follower(IDConstants.rightPivotID, true));
		
		m_absEncoder = new DutyCycleEncoder(PortConstants.pivotAbsEncoderID);
		m_pivotController = new PIDController(motorConfigs.kP, motorConfigs.kI, motorConfigs.kD);
		m_pivotController.setSetpoint(m_absEncoder.getAbsolutePosition());
		m_pivotController.setTolerance(0.015);
		
	}

	public void periodic() {
		pivotTable.putValue("Absolute Encoder Position",
				NetworkTableValue.makeDouble(m_absEncoder.getAbsolutePosition()));
		if(m_absEncoder.getAbsolutePosition() != 0) {
			double output = m_pivotController.calculate(m_absEncoder.getAbsolutePosition());
			if(m_usePID) {
				if(m_pivotController.atSetpoint()){
					m_motorR.setControl(new DutyCycleOut(0));
				}
				else{
					m_motorR.setControl(new DutyCycleOut(output));
				}
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
		pivotTable.putValue("P Proportion", NetworkTableValue.makeDouble(m_pivotController.getPositionError()*m_pivotController.getP()));
		pivotTable.putValue("D Proportion", NetworkTableValue.makeDouble(m_pivotController.getVelocityError()*m_pivotController.getD()));


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
			m_pivotController.setSetpoint(m_absEncoder.getAbsolutePosition());//TODO constrain setpoint to within limit switches
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
		m_pivotController.setSetpoint(position); //TODO constrain setpoint to within limit switches--make setpoint safe method
		System.out.println(position);
	};
	//TODO angle to sucesful shot, amp, speaker, and podium setpoint
}