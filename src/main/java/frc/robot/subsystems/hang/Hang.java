package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm.ArmControlType;
import frc.robot.util.IOUtils;

public class Hang extends SubsystemBase {
    private static Hang instance = null; //new Hang();

    public static Hang getInstance() {
        return instance; 
    }

    //Initializing velocity variable
    private double m_velocity = 0;
    private double m_hangFeedforward;
    private double m_manualVelocity = 0; 
    public double m_targetPosition;
    
    //Initialize Motors
    private CANSparkMax m_HangMotor = new CANSparkMax(Constants.HangConstants.kHangMotorID, MotorType.kBrushless);

    //Encoder Initialize
    private RelativeEncoder m_HangEncoder = m_HangMotor.getEncoder();

    private PIDController m_HangPidController = new PIDController(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);

    private HangControlType m_HangControlType;


    private Hang() {
        setupMotors();
    }

    public void setupMotors() {
        //Sets motors inverted
        m_HangMotor.setInverted(Constants.HangConstants.kMotorInverted);
        // m_HangMotorLeft.setInverted(Constants.HangConstants.kLeftMotorInverted);
        
        //Sets Smart Limits
        m_HangMotor.setSmartCurrentLimit(Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for right Encoders
        m_HangEncoder.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoder.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        m_HangMotor.setIdleMode(IdleMode.kBrake);

        m_HangPidController.setTolerance(Constants.HangConstants.kHangTolerance);
        //Sets  limits for Right and Left Motors
        //Right Motors
        m_HangMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.HangConstants.kFowardHangSoftLimit);
        m_HangMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.HangConstants.kReverseHangSoftLimit);
        m_HangMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_HangMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        m_HangPidController.setP(Constants.HangConstants.kP);
        m_HangPidController.setI(Constants.HangConstants.kI);
        m_HangPidController.setD(Constants.HangConstants.kD);
        
        m_HangMotor.burnFlash();
    }

    @Override
    public void periodic() {
        m_velocity = m_HangPidController.calculate(getHangPosition(), m_targetPosition);
        if (m_HangMotor.getOutputCurrent()>100 && velocityWithinTolerance(0)) {
           m_velocity = 0;
        }
        m_HangMotor.set(m_velocity);
        
        //Constantly sends logs to Smart Dashboard
        doSendables();

          if (m_HangControlType == HangControlType.PID) {

            m_velocity = m_HangPidController.calculate(getHangPosition(), m_targetPosition) 
                                + m_hangFeedforward;
        }
        
        // velocity controlled manually
        else if (m_HangControlType == HangControlType.MANUAL) {
            m_velocity = m_manualVelocity; 
        }
    }

    public void doSendables() {
        // Add logging for hang (eg. encoder positions, velocity, etc. )
        IOUtils.set("Hang Target Velocity (m/s)", m_velocity);
        IOUtils.set("Right Hang Current Velocity (m/s)", m_HangEncoder.getVelocity());
        IOUtils.set("Right Hang Current Position", m_HangEncoder.getPosition());

        // SmartDashboard.putBoolean("Hang RM Inverted", m_HangMotorRight.getInverted());
        // SmartDashboard.putBoolean("Hang LM Inverted", m_HangMotorLeft.getInverted());
        // IOUtils.set("Hang Soft Limit ForwardRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kReverse));
        // IOUtils.set("Hang Soft Limit ForwardLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kReverse));
        
    }

    public void setSpeed(double speed) {
        //Method to change speed
        this.m_velocity = speed;
    }
    public void setManualVelocity(double velocity){
        this.m_manualVelocity = velocity;
    }



    public double getHangPosition() {
        return m_HangEncoder.getPosition();
    }

    public void setPosition(double targetposition) {
        this.m_targetPosition = targetposition;
    }

    public boolean isAtPosition() {
        return m_HangPidController.atSetpoint();
    }

    public boolean velocityWithinTolerance(double TargetSpeed) {
        return Math.abs(m_HangEncoder.getVelocity()-TargetSpeed) <= Constants.HangConstants.kHangTolerance;
    }

    public enum HangControlType {
        MANUAL, 
        PID
    }
    

}