package frc.robot.commands;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.FrontVision;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;

public class AutomaticallyMoveToPiece {

    public static Command waitForVision(FrontVision frontVision) {
        return Commands.waitUntil(() -> {
            var result = frontVision.getLatestVisionResult();
            return result == null ? false : result.hasTargets(); 
        }); 
    }

    public static Command automaticallyMoveToPiece(DriverController driverController, Drivetrain drivetrain, FrontVision frontVision) {
        var result = frontVision.getLastSuccessfulResult();
        if (!result.hasTargets()) return Commands.none();
        
        double angleToTurn = frontVision.getNoteYaw();
        double actualAngle = angleToTurn + drivetrain.getPose().getRotation().getDegrees();

        return new ParallelRaceGroup(Commands.run(() -> {

            ChassisSpeeds driverInput = driverController.getDesiredChassisSpeeds();

            driverInput.vyMetersPerSecond = 0; 

            drivetrain.swerveDriveFieldRel(new ChassisState(
                driverInput.vxMetersPerSecond * Math.cos(Math.toRadians(actualAngle)) - driverInput.vyMetersPerSecond * Math.sin(Math.toRadians(actualAngle)), 
                driverInput.vxMetersPerSecond * Math.sin(Math.toRadians(actualAngle)) + driverInput.vyMetersPerSecond * Math.cos(Math.toRadians(actualAngle)), 
                Math.toRadians(actualAngle), true
                ), false, false);
        }, drivetrain), AutomationCommands.autoIntakeCommand()); // Any processing before turning to that angle
    }

    public static Command autoMoveToPiece(Drivetrain drivetrain, FrontVision frontVision) {

        return Commands.defer(() -> {
            Transform2d noteTransform = frontVision.getNotePose(); 

            Pose2d currentPose = drivetrain.getPose(); 

            Pose2d notePose = currentPose.transformBy(noteTransform);

            PathConstraints constraints = new PathConstraints(
                Constants.Trajectory.kMaxVelocityMetersPerSecond, 
                Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared,
                Constants.Trajectory.kMaxAngularVelocityRadiansPerSecond, 
                Constants.Trajectory.kMaxAngularAccelerationRadiansPerSecondSquared
            );

            List<Translation2d> poses = PathPlannerPath.bezierFromPoses(currentPose, notePose); 

            PathPlannerPath path = new PathPlannerPath(poses, constraints, new GoalEndState(0, notePose.getRotation(), true));
            
            path.preventFlipping = true; 

            return new ParallelRaceGroup(
                AutoBuilder.followPath(path),
                AutomationCommands.autoIntakeCommand()
            );
        }, Set.of(drivetrain)); 

    }
}
