package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.IOUtils;

public class Hang extends SubsystemBase {
    private static Hang instance = new Hang();

    public static Hang getInstance() {
        return instance; 
    }

    //Initializing velocity variable
    private double m_manualVelocity = 0; 
    public double m_targetPosition = 0;
    
    //Initialize Motors
    private CANSparkMax m_HangMotor = new CANSparkMax(Constants.HangConstants.kHangMotorID, MotorType.kBrushless);

    //Encoder Initialize
    private RelativeEncoder m_HangEncoder = m_HangMotor.getEncoder();

    private PIDController m_HangPidController = new PIDController(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);

    private HangControlType m_HangControlType = HangControlType.MANUAL;


    private Hang() {
        setupMotors();
    }

    public void setupMotors() {
        //Sets motors inverted
        m_HangMotor.setInverted(Constants.HangConstants.kMotorInverted);
        
        //Sets Smart Limits
        m_HangMotor.setSmartCurrentLimit(20, Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for encoders
        m_HangEncoder.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoder.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        m_HangMotor.setIdleMode(IdleMode.kBrake);

        setPID(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);
        
        m_HangMotor.burnFlash();
    }

    public void setPID(double p, double i, double d) {
        m_HangPidController.setP(p);
        m_HangPidController.setI(i);
        m_HangPidController.setD(d);
    }

    @Override
    public void periodic() {

        double velocity = 0; 

        //Constantly sends logs to Smart Dashboard
        doSendables();

          if (m_HangControlType == HangControlType.PID) {
            velocity = m_HangPidController.calculate(getHangPosition(), m_targetPosition);
        }
        
        // velocity controlled manually
        else if (m_HangControlType == HangControlType.MANUAL) {
            velocity = m_manualVelocity; 
        }

        m_HangMotor.set(velocity);
    }

    public void doSendables() {
        // Add logging for hang (eg. encoder positions, velocity, etc. )
        IOUtils.set("Hang Current Velocity", m_HangEncoder.getVelocity());
        IOUtils.set("Hang Current Position", m_HangEncoder.getPosition());
        IOUtils.set("Hang Target Position", m_targetPosition);
        IOUtils.set("Hang Error", getError());
        IOUtils.set("Hang Current", m_HangMotor.getOutputCurrent());
        IOUtils.set("Hang Bus Voltage", m_HangMotor.getBusVoltage());
        IOUtils.set("Hang Output", m_HangMotor.getAppliedOutput());
        
    }
    public void setManualVelocity(double velocity) {
        this.m_manualVelocity = velocity;
    }

    public double getHangPosition() {
        return m_HangEncoder.getPosition();
    }

    public void setTargetPosition(double targetPosition) {
        this.m_targetPosition = targetPosition;
    }

    public boolean isAtPosition() {
        return getError() <= Constants.HangConstants.kHangTolerance; 
    }

    public double getError() {
        return Math.abs(m_HangEncoder.getPosition() - this.m_targetPosition); 
    }

    public double getHangVelocity() {
        return m_HangEncoder.getVelocity();
    }

    public double getMotorCurrent() {
        return m_HangMotor.getOutputCurrent();
    }

    public void setHangMode(HangControlType type) {
        this.m_HangControlType = type;
    }

    public enum HangControlType {
        MANUAL, 
        PID
    }
}