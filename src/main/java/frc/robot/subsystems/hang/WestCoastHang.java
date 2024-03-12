package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.util.IOUtils;

public class WestCoastHang extends IHang {
    private static WestCoastHang instance = new WestCoastHang();

    public static WestCoastHang getInstance() {
        return instance;
    }

//Initializing velocity variable
    private double m_manualVelocity = 0; 
    public double m_targetPosition = 0;
    
    //Initialize Motors
    private CANSparkMax m_rightHangMotor = new CANSparkMax(Constants.WestCoastHangConstants.kRightHangMotorID, MotorType.kBrushless);
    private CANSparkMax m_leftHangMotor = new CANSparkMax(Constants.WestCoastHangConstants.kLeftHangMotorID, MotorType.kBrushless);

    //Encoder Initialize
    private RelativeEncoder m_rightHangEncoder = m_rightHangMotor.getEncoder();

    private PIDController m_rightHangPidController = new PIDController(Constants.WestCoastHangConstants.kP, Constants.WestCoastHangConstants.kI, Constants.WestCoastHangConstants.kD);

    private HangControlType m_HangControlType = HangControlType.MANUAL;

    private WestCoastHang() {
        setupMotors();
    }

    public void setupMotors() {
        //Sets motors inverted
        m_rightHangMotor.setInverted(Constants.WestCoastHangConstants.kRightMotorInverted);
        
        //Sets Smart Limits
        m_rightHangMotor.setSmartCurrentLimit(Constants.WestCoastHangConstants.kHangStallCurrent, Constants.WestCoastHangConstants.kHangMotorLimit);
        m_leftHangMotor.setSmartCurrentLimit(Constants.WestCoastHangConstants.kHangStallCurrent, Constants.WestCoastHangConstants.kHangMotorLimit);

        //Conversion Factors for encoders
        m_rightHangEncoder.setPositionConversionFactor(Constants.WestCoastHangConstants.kPositionConversionFactor);
        m_rightHangEncoder.setVelocityConversionFactor(Constants.WestCoastHangConstants.kVelocityConversionFactor);

        m_rightHangMotor.setIdleMode(IdleMode.kBrake);
        m_leftHangMotor.setIdleMode(IdleMode.kBrake);

        m_leftHangMotor.follow(m_rightHangMotor, Constants.WestCoastHangConstants.kLeftMotorInverted);

        setPID(Constants.WestCoastHangConstants.kP, Constants.WestCoastHangConstants.kI, Constants.WestCoastHangConstants.kD);
        
        m_rightHangMotor.burnFlash();
        m_leftHangMotor.burnFlash();
    }

    public void setPID(double p, double i, double d) {
        m_rightHangPidController.setP(p);
        m_rightHangPidController.setI(i);
        m_rightHangPidController.setD(d);
    }

    @Override
    public void periodic() {

        double velocity = 0; 

        //Constantly sends logs to Smart Dashboard
        doSendables();

          if (m_HangControlType == HangControlType.PID) {
            velocity = m_rightHangPidController.calculate(getHangPosition(), m_targetPosition);
        }
        
        // velocity controlled manually
        else if (m_HangControlType == HangControlType.MANUAL) {
            velocity = m_manualVelocity; 
        }

        m_rightHangMotor.set(velocity);
    }

    public void setManualVelocity(double velocity) {
        this.m_manualVelocity = velocity;
    }

    public double getHangPosition() {
        return m_rightHangEncoder.getPosition();
    }

    public void setHangPosition(double targetPosition) {
        this.m_targetPosition = targetPosition;
    }

    public boolean isAtPosition() {
        return getError() <= Constants.HangConstants.kHangTolerance; 
    }

    public double getError() {
        return Math.abs(m_rightHangEncoder.getPosition() - this.m_targetPosition); 
    }

    public double getHangVelocity() {
        return m_rightHangEncoder.getVelocity();
    }

    public double getMotorCurrent() {
        return m_rightHangMotor.getOutputCurrent();
    }
 
    public void setHangMode(HangControlType type) {
        this.m_HangControlType = type;
    }

    public void doSendables() {
        // Add logging for hang (eg. encoder positions, velocity, etc. )
        IOUtils.set("Hang Current Velocity", m_rightHangEncoder.getVelocity());
        IOUtils.set("Hang Current Position", m_rightHangEncoder.getPosition());
        IOUtils.set("Hang Target Position", m_targetPosition);
        IOUtils.set("Hang Error", getError());
        IOUtils.set("Hang Current", m_rightHangMotor.getOutputCurrent());
        IOUtils.set("Hang Bus Voltage", m_rightHangMotor.getBusVoltage());
        IOUtils.set("Hang Output", m_rightHangMotor.getAppliedOutput());
    }    

}
