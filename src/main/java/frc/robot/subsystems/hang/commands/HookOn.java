package frc.robot.subsystems.hang.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.IHang;
import frc.robot.subsystems.hang.WestCoastHang;
import frc.robot.subsystems.hang.IHang.HangControlType;

public class HookOn extends Command {
    IHang m_hang;

    public HookOn(WestCoastHang hang) {
        this.m_hang = hang;
        addRequirements(hang);
    }

    public void initialize() {
        m_hang.setHangMode(HangControlType.MANUAL);
        
    }

    public void execute() {

    }

    public boolean isFinished() {
        return true;
    }

    public void end() {

    }
}
