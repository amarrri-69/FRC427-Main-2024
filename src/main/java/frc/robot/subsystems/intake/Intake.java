package frc.robot.subsystems.intake;

import java.util.Set;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 /*
  * Notes about how Intake works
  - theres 2 main parts, The SHOOTER (intakMoterShootTOP/BOTTOM) and the SUCKER (outtakeMotorSuck)
  - the shooter has 2 motors Top and Bottom (teh bottom one follows the top) and the sucker has only one 
  - (the naming conventions are a little swapped thats my fault yo...)
  - so the sucker, SUCKS the rings from the ground and puts them into the shooter.
  - Then the shooter takes the rings from the sucker and shoots them into whatever it needs
  */

public class Intake extends SubsystemBase {

    private static Intake instance = new Intake();

    public static Intake getInstance() {
        return instance; 
    }

    // desire speed
    private double m_desiredTopSpeed;
    private double m_desiredBottomSpeed; 
    
    // establishes the motors for shooter and sucker. Also establishes the beambreak.
    CANSparkMax m_intakeMotorShootTop = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorShootTopId, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoderShootTop = m_intakeMotorShootTop.getEncoder();

    CANSparkMax m_intakeMotorShootBottom = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorShootBottomId, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoderShootBottom = m_intakeMotorShootBottom.getEncoder();

    CANSparkMax m_outtakeMotorSuck = new CANSparkMax(Constants.IntakeConstants.kOuttakeMotorSuckId, MotorType.kBrushless);
    RelativeEncoder m_outtakeEncoderSuck = m_outtakeMotorSuck.getEncoder();

    SparkPIDController m_outtakeTopController = m_intakeMotorShootTop.getPIDController();
    SparkPIDController m_outtakeBottomController = m_intakeMotorShootBottom.getPIDController();


    // beambreak checks for if theres a note in the intake
    DigitalInput m_BeamBreak = new DigitalInput(Constants.IntakeConstants.kBeamBreakId); 

    
    private Intake() {
        setupMotors();
        setupControllers();

    }
    public void setupMotors() {
        // sets limits for all the motors and has the bottom shoot motor, follow the top
        m_intakeMotorShootTop.setInverted(Constants.IntakeConstants.kShootTopIntakeInverted);
        m_intakeMotorShootTop.setSmartCurrentLimit(Constants.IntakeConstants.kShootTopMotorlimit);
        m_intakeEncoderShootTop.setVelocityConversionFactor(Constants.IntakeConstants.kShootVelocityConversionFactor); 
        m_intakeMotorShootTop.enableVoltageCompensation(12); 

        m_intakeMotorShootBottom.setInverted(Constants.IntakeConstants.kShootBottomIntakeInverted);
        m_intakeMotorShootBottom.setSmartCurrentLimit(Constants.IntakeConstants.kShootBottomMotorlimit);
        m_intakeEncoderShootBottom.setVelocityConversionFactor(Constants.IntakeConstants.kShootVelocityConversionFactor); 
        m_intakeMotorShootBottom.enableVoltageCompensation(12); 

        m_outtakeMotorSuck.setInverted(Constants.IntakeConstants.kSuckOuttakeInverted);
        m_outtakeMotorSuck.setSmartCurrentLimit(Constants.IntakeConstants.kSuckOuttakeMotorLimit);
        m_outtakeEncoderSuck.setVelocityConversionFactor(Constants.IntakeConstants.kIntakeVelocityConversionFactor); 
    }

    public void setupControllers() {
        m_outtakeTopController.setP(Constants.IntakeConstants.kTopP);
        m_outtakeTopController.setI(Constants.IntakeConstants.kTopI);
        m_outtakeTopController.setD(Constants.IntakeConstants.kTopD);
        m_outtakeTopController.setFF(Constants.IntakeConstants.kTopFF);

        m_outtakeBottomController.setP(Constants.IntakeConstants.kBottomP);
        m_outtakeBottomController.setI(Constants.IntakeConstants.kBottomI);
        m_outtakeBottomController.setD(Constants.IntakeConstants.kBottomD);
        m_outtakeBottomController.setFF(Constants.IntakeConstants.kBottomFF);
    }

    public void periodic() {
        // code inside here will run repeatedly while the robot is on
        m_outtakeTopController.setReference(m_desiredTopSpeed, CANSparkBase.ControlType.kVelocity);
        m_outtakeBottomController.setReference(m_desiredBottomSpeed, CANSparkBase.ControlType.kVelocity);
        doSendables();
    }
    //so intaking the ring is sucking it
    public void intakeRing(double speed) {
        m_outtakeMotorSuck.set(speed);
    }
    //and outtaking the ring is shooting it
    public void outtakeRing(double speed) {
        // m_intakeMotorShootTop.set(speed);
        this.m_desiredTopSpeed = speed;
        this.m_desiredBottomSpeed = speed + 50; 
    }

    public void outtakeTop(double speed) {
        this.m_desiredTopSpeed = speed; 
    }

    public void outtakeBottom(double speed) {
        this.m_desiredBottomSpeed = speed + 50; 
    }
    //beambreak is a scanner that checks if a ring is inside the whole intake
    public boolean beamBreakHit() { 
        return !m_BeamBreak.get();
    }
    public void stopSuck() {
        m_outtakeMotorSuck.set(0); 
    }
    
    public void stopShoot() {
        // m_intakeMotorShootTop.set(0);
        outtakeRing(0);
    }

    public boolean atDesiredShootSpeed() {
        return (Math.abs(m_intakeEncoderShootTop.getVelocity() - m_desiredTopSpeed) <= Constants.IntakeConstants.kTolerance) && (Math.abs(m_intakeEncoderShootBottom.getVelocity() - m_desiredBottomSpeed) <= Constants.IntakeConstants.kTolerance); 
    }

     public void doSendables() { 
        SmartDashboard.putNumber("Suck Speed (m/s)", m_outtakeEncoderSuck.getVelocity());
        SmartDashboard.putNumber("Shoot Top Speed (m/s)", m_intakeEncoderShootTop.getVelocity());
        SmartDashboard.putNumber("Shoot Bottom Speed (m/s)", m_intakeEncoderShootBottom.getVelocity());
        SmartDashboard.putNumber("Outtake Desired Speed", m_desiredTopSpeed); 
        SmartDashboard.putBoolean("Shoot At Desired Speed", atDesiredShootSpeed()); 
        SmartDashboard.putBoolean("Beam Break Hit (t/f)", beamBreakHit());
        SmartDashboard.putNumber("Indexer Current", m_outtakeMotorSuck.getOutputCurrent());
    }

    public Command tuneTopPID() {
        return Commands.defer(() -> {
            SmartDashboard.putNumber("TuneShooter/P", Constants.IntakeConstants.kTopP); 
            SmartDashboard.putNumber("TuneShooter/I", Constants.IntakeConstants.kTopI); 
            SmartDashboard.putNumber("TuneShooter/D", Constants.IntakeConstants.kTopD); 
            SmartDashboard.putNumber("TuneShooter/FF", Constants.IntakeConstants.kTopFF); 
            SmartDashboard.putNumber("TuneShooter/ShooterDesiredSpeed", 0); 
            return Commands.run(() -> {
                double desSpeed = SmartDashboard.getNumber("TuneShooter/ShooterDesiredSpeed", 0); 

                m_outtakeTopController.setP(SmartDashboard.getNumber("TuneShooter/P", Constants.IntakeConstants.kTopP)); 
                m_outtakeTopController.setI(SmartDashboard.getNumber("TuneShooter/I", Constants.IntakeConstants.kTopI)); 
                m_outtakeTopController.setD(SmartDashboard.getNumber("TuneShooter/D", Constants.IntakeConstants.kTopD)); 
                m_outtakeTopController.setFF(SmartDashboard.getNumber("TuneShooter/FF", Constants.IntakeConstants.kTopFF)); 

                outtakeTop(desSpeed);
            }); 
        }, Set.of(Intake.getInstance())); 
    }

    public Command tuneBottomPID() {
        return Commands.defer(() -> {
            SmartDashboard.putNumber("TuneShooter/BotP", Constants.IntakeConstants.kBottomP); 
            SmartDashboard.putNumber("TuneShooter/BotI", Constants.IntakeConstants.kBottomI); 
            SmartDashboard.putNumber("TuneShooter/BotD", Constants.IntakeConstants.kBottomD); 
            SmartDashboard.putNumber("TuneShooter/BotFF", Constants.IntakeConstants.kBottomFF); 
            SmartDashboard.putNumber("TuneShooter/ShooterDesiredSpeed", 0); 
            return Commands.run(() -> {
                double desSpeed = SmartDashboard.getNumber("TuneShooter/ShooterDesiredSpeed", 0); 

                m_outtakeBottomController.setP(SmartDashboard.getNumber("TuneShooter/BotP", Constants.IntakeConstants.kBottomP)); 
                m_outtakeBottomController.setI(SmartDashboard.getNumber("TuneShooter/BotI", Constants.IntakeConstants.kBottomI)); 
                m_outtakeBottomController.setD(SmartDashboard.getNumber("TuneShooter/BotD", Constants.IntakeConstants.kBottomD)); 
                m_outtakeBottomController.setFF(SmartDashboard.getNumber("TuneShooter/BotFF", Constants.IntakeConstants.kBottomFF)); 

                outtakeBottom(desSpeed);
            }); 
        }, Set.of(Intake.getInstance())); 
    }
}