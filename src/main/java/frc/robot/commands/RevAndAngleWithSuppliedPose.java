package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Rotation2dStruct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;

public class RevAndAngleWithSuppliedPose extends Command {

    public Arm arm;
    public Intake intake;
    public Optional<Alliance> optAlliance;
    public Supplier<Pose2d> targetPose;
    private Consumer<Rotation2d> rotation; 
    private Pose2d lastPose = null; 

    public RevAndAngleWithSuppliedPose(Arm arm, Intake intake, Supplier<Pose2d> targetPose, Consumer<Rotation2d> rotationConsumer) {
        this.arm = arm;
        this.intake = intake;
        this.targetPose = targetPose;
        this.rotation = rotationConsumer;

        addRequirements(arm, intake);
    }

    public void initialize() {
        this.optAlliance = DriverStation.getAlliance();

        CommandScheduler.getInstance().schedule(SetShooterSpeed.indexNote(intake));
    }

    public void execute() {
        lastPose = targetPose.get() != null ? targetPose.get() : lastPose; 

        if (lastPose == null) return; 
        
        ShootAnywhereResult res = ShootAnywhere.getShootValues(lastPose); 

        if (res == null) return; 

        arm.goToAngle(res.getArmAngleDeg());
        intake.outtakeTop(res.getOuttakeTopSpeed());
        intake.outtakeBottom(res.getOuttakeBottomSpeed());
        rotation.accept(Rotation2d.fromDegrees(res.getDriveAngleDeg()));
    }

    public boolean isFinished() {
        return optAlliance.isEmpty() || Constants.GeneralizedReleaseConstants.readyToShootAuto.getAsBoolean();
    }

    public static Command createCommand(Arm arm, Intake intake, Supplier<Pose2d> targetPose, Consumer<Rotation2d> rotation) {
        return Commands.defer(() -> new RevAndAngleWithSuppliedPose(arm, intake, targetPose, rotation), Set.of(arm, intake)).withTimeout(2); 
    }

    public static Command createRotationCommand(Supplier<Pose2d> pose, Consumer<Rotation2d> setRotation) {
        return Commands.defer(() -> {
            Pose2d targetPose = pose.get(); 
            // Optional<Alliance> optAlliance = DriverStation.getAlliance(); 
            // if (optAlliance.isPresent() && optAlliance.get() == Alliance.Red) {
            //     targetPose = new Pose2d(Constants.Vision.kAprilTagFieldLayout.getFieldLength() - targetPose.getX(), targetPose.getY(), targetPose.getRotation());
            // }
            ShootAnywhereResult res = ShootAnywhere.getShootValues(targetPose);
            return Commands.startEnd(
                () -> {
                    setRotation.accept(Rotation2d.fromDegrees(res.getDriveAngleDeg()));
                }, 
                () -> {
                    setRotation.accept(null);
                }
            );
        }, Set.of()); 
    }
} 
