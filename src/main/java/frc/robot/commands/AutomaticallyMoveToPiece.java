package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnBy;
import frc.robot.subsystems.vision.FrontVision;
import frc.robot.util.DriverController;

public class AutomaticallyMoveToPiece {

    public static Command automaticallyMoveToPiece(DriverController driverController, Drivetrain drivetrain, FrontVision frontVision) {
        var result = frontVision.getLatestVisionResult();
        if (!result.hasTargets()) return Commands.none();
        
        double angleToTurn = frontVision.getNoteRotation();

        return new ParallelRaceGroup(new TurnBy(drivetrain, angleToTurn).andThen(Commands.run(() -> {
            drivetrain.swerveDriveRobotCentric(new ChassisSpeeds(driverController.getDesiredChassisSpeeds().vxMetersPerSecond, 0, 0));
        }, drivetrain)), AutomationCommands.autoIntakeCommand()); // Any processing before turning to that angle
    }
}
