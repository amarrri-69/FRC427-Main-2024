package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IHang extends SubsystemBase {

    public abstract void setPID(double p, double i, double d);
    public abstract void setManualVelocity(double velocity);
    public abstract double getHangPosition(); 
    public abstract void setHangPosition(double targetPosition);
    public abstract boolean isAtPosition();
    public abstract double getHangVelocity();
    public abstract double getMotorCurrent();
    public abstract void setHangMode(HangControlType type);

    public enum HangControlType {
        MANUAL,
        PID
    }

}

