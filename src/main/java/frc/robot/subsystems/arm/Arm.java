package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class Arm extends SubsystemBase {
    
    private static Arm instance = new Arm(); 

    public static Arm getInstance() {
        return instance; 
    }
    
    // Define and initiate variables for arm
    private double m_targetPosition = Constants.ArmConstants.kTravelPosition;
    private double m_manualVelocity = 0;
    

    private DigitalInput m_limitSwitch = new DigitalInput(Constants.ArmConstants.kLimitSwitchId);

    private CANSparkMax m_armMotorRight = new CANSparkMax(Constants.ArmConstants.kArmMotorRightId, MotorType.kBrushless);
    private CANSparkMax m_armMotorLeft = new CANSparkMax(Constants.ArmConstants.kArmMotorLeftId, MotorType.kBrushless);

    // encoder from the right motor
    private AbsoluteEncoder m_armAbsoluteEncoder = m_armMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder m_armRelativeEncoder = m_armMotorRight.getEncoder();
    
    private PIDController m_armPIDController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);
    
    public ArmFeedforward m_armFeedforward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG, Constants.ArmConstants.kV, Constants.ArmConstants.kA);

    private ArmControlType m_ArmControlType = Arm.ArmControlType.PID;


    private Arm() {
        setupEncoders();
        setupControllers();
        setupMotors();
    }

    // motor config
    public void setupMotors() {

        m_armMotorRight.setIdleMode(IdleMode.kBrake);
        m_armMotorLeft.setIdleMode(IdleMode.kBrake);

        m_armMotorRight.setInverted(Constants.ArmConstants.kRightMotorInverted);
        
        m_armMotorRight.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);
        m_armMotorLeft.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);
        
        // left arm motor would follow right arm  motor's voltage intake 
        m_armMotorLeft.follow(m_armMotorRight, Constants.ArmConstants.kLeftMotorInverted);

        m_armMotorLeft.burnFlash(); 
        m_armMotorRight.burnFlash();
    }

    //encoder config
    public void setupEncoders() {
        // conversion factors
        m_armAbsoluteEncoder.setPositionConversionFactor(Constants.ArmConstants.kAbsPositionConversionFactor);
        m_armAbsoluteEncoder.setVelocityConversionFactor(Constants.ArmConstants.kAbsVelocityConversionFactor);

        m_armRelativeEncoder.setPositionConversionFactor(Constants.ArmConstants.kRelativePositionConversionFactor); 
        m_armRelativeEncoder.setVelocityConversionFactor(Constants.ArmConstants.kRelativeVelocityConversionFactor); 
        
        m_armMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
        // in tina we trust
        // m_armRelativeEncoder.setPosition(m_armEncoderRight.getPosition());
    }

    // pid controller config
    public void setupControllers() {
        // position error on which it is tolerable
        m_armPIDController.setTolerance(Constants.ArmConstants.kTolerance);
    }

    public void periodic() {
        doSendables();
        
        double impendingVelocity = 0; 

        if (m_ArmControlType == ArmControlType.PID) {

            impendingVelocity = m_armPIDController.calculate(getAngle(), m_targetPosition) 
                                + m_armFeedforward.calculate(Math.toRadians(getAngle()), 0);
        }
        
        // velocity controlled manually
        else if (m_ArmControlType == ArmControlType.MANUAL) {
            impendingVelocity = m_manualVelocity;
        }

        // if the arm is not reaching beyond the limit switch,
        // or if the arm is not reaching beyond 100 degs, 
        // arm is allowed to move 
        // forward defined as away from the limit switch.
        boolean passReverseSoftLimit = reverseSoftLimit() && impendingVelocity < 0;
        boolean passForwardSoftLimit = forwardSoftLimit() && impendingVelocity > 0;

        // if (!passReverseSoftLimit && !passForwardSoftLimit) {
            m_armMotorRight.set(impendingVelocity); 
        // }

        SmartDashboard.putNumber("Impending Velocity (m/s)", impendingVelocity);
        SmartDashboard.putBoolean("Pass Reverse Soft Limit", passReverseSoftLimit);
        SmartDashboard.putBoolean("Pass Forward Soft Limit", passForwardSoftLimit);
    }

    public boolean reverseSoftLimit() {
        // return (getLimitSwitchValue() || getAngle() < Constants.ArmConstants.kReverseSoftLimit);
        return (getAngle() < Constants.ArmConstants.kReverseSoftLimit);
    }

    public boolean forwardSoftLimit() {
        return getAngle() > Constants.ArmConstants.kForwardSoftLimit;
    }

    public boolean getLimitSwitchValue() {
        return m_limitSwitch.get(); 
    }

    // public void setKG(double kG) {
    //     this.m_kG = kG;
    // }

    // public void setKS(double kS) {
    //     this.m_kS = kS;
    // }

    public double getError() {
        return m_armPIDController.getPositionError();
    }

    public void goToAngle(double angle) {
        this.m_targetPosition = angle;
    }

    public double getAngle() {
        return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(m_armAbsoluteEncoder.getPosition())));
    }


    public void setSpeed(double speed) {
        this.m_manualVelocity = speed;
    }

    public boolean isAtAngle() {
        return m_armPIDController.atSetpoint();
        
    }

    public void setControlType(ArmControlType type) {
        this.m_ArmControlType = type;
    }

    public void setPID(double p, double i, double d) {
        this.m_armPIDController.setPID(p, i, d);
    }

    public ArmControlState getArmControlState() {
        if (m_targetPosition == Constants.ArmConstants.kGroundPosition) {
            return ArmControlState.GROUND;
        }
        else if (m_targetPosition == Constants.ArmConstants.kTravelPosition) {
            return ArmControlState.TRAVEL;
        }
        else if (m_targetPosition == Constants.ArmConstants.kAmpPosition) {
            return ArmControlState.AMP;
        }
        else if (m_targetPosition == Constants.ArmConstants.kSpeakerPosition) {
            return ArmControlState.SPEAKER;
        }
        else {
            return ArmControlState.CUSTOM;
        }
    }

    public enum ArmControlType {
        MANUAL, 
        PID
    }

    public enum ArmControlState {
        GROUND,
        TRAVEL,
        AMP,
        SPEAKER, 
        CUSTOM
    }

    // add logging for arm 
    // are units correct?
    public void doSendables() {
        SmartDashboard.putNumber("Arm Target Position (deg)", m_targetPosition);
        SmartDashboard.putNumber("Arm Position (deg)", getAngle()); 
        SmartDashboard.putNumber("Arm Absolute Position (deg)", m_armAbsoluteEncoder.getPosition()); 
        SmartDashboard.putNumber("Arm Velocity (deg/sec)", m_armAbsoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Error (deg)", getError());
        SmartDashboard.putBoolean("Arm At Set Point", isAtAngle());
        SmartDashboard.putString("Arm Control Type", m_ArmControlType.toString());
        SmartDashboard.putString("Arm Control State", getArmControlState().toString());
        SmartDashboard.putNumber("left volt", m_armMotorLeft.get()); 
        SmartDashboard.putNumber("right volt", m_armMotorRight.get()); 

        SmartDashboard.putBoolean("Arm Limit Switch", getLimitSwitchValue());
        // SmartDashboard.putBoolean("left inverted", m_armMotorLeft.getInverted()); 
        // SmartDashboard.putBoolean("right inverted", m_armMotorRight.getInverted()); 
    }
}
