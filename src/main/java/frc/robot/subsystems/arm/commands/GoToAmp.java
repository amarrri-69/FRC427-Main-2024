package frc.robot.subsystems.arm.commands;

import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

// controlled by pid 
public class GoToAmp extends GoToAngle {
    Arm m_arm;
    
    public GoToAmp(Arm arm) {
        super(arm, Constants.ArmConstants.kAmpPosition);
    }

}