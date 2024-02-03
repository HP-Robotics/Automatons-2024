package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private TalonFX m_motorR;
    private TalonFX m_motorL;

    /** Creates a new ExampleSubsystem. */
    public PivotSubsystem() {
        double speed = 0.0;
        m_motorR = new TalonFX(PivotConstants.motorRID);
        m_motorL = new TalonFX(PivotConstants.motorLID);
        Slot0Configs motorConfigs = new Slot0Configs();
        motorConfigs.kP = SmartDashboard.getNumber("kP", PivotConstants.kP);
        motorConfigs.kI = SmartDashboard.getNumber("kI", PivotConstants.kI);
        motorConfigs.kD = SmartDashboard.getNumber("kD", PivotConstants.kD);
        m_motorR.getConfigurator().apply(motorConfigs);
        m_motorL.getConfigurator().apply(motorConfigs);
    }

    public void periodic() {
        // This method will be called once per scheduler run
    };

    public void setSpeed() {

    }

    public void movePivot(double output, boolean manual) {
        if (manual) {
            m_motorR.setControl(new DutyCycleOut(output));
            m_motorL.setControl(new DutyCycleOut(output));
        } else {
            m_motorR.setControl(new PositionDutyCycle(output));
            m_motorL.setControl(new PositionDutyCycle(output));
        }
    };
}