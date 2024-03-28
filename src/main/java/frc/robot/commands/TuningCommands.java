package frc.robot.commands;

import java.util.Set; 

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootAnywhere.ShootAnywhereResult;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;
import frc.robot.util.IOUtils;

public class TuningCommands {
  public static Command tuneShooting(Drivetrain drivetrain, Arm arm, Intake intake) {
    return Commands.defer(() -> {

        boolean usePresetAngle = IOUtils.getBoolean("TuneShooting/Use Preset Angle");
        boolean usePresetSpeed = IOUtils.getBoolean("TuneShooting/Use Preset Speed"); 

        ShootAnywhereResult res = ShootAnywhere.getShootValues(drivetrain.getPose());

        
        if (res == null) return Commands.none();

        IOUtils.set("TuneShooting/Preset Angle", res.getArmAngleDeg()); 
        IOUtils.set("TuneShooting/Preset Top Speed", res.getOuttakeTopSpeed());
        IOUtils.set("TuneShooting/Preset Bottom Speed", res.getOuttakeBottomSpeed()); 
        
        double angle = usePresetAngle ? res.getArmAngleDeg() : IOUtils.getNumber("TuneShooting/Desired Angle"); 
        double topSpeed = usePresetSpeed ? res.getOuttakeTopSpeed() : IOUtils.getNumber("TuneShooting/Desired Top Speed");
        double bottomSpeed = usePresetSpeed ? res.getOuttakeTopSpeed() : IOUtils.getNumber("TuneShooting/Desired Bottom Speed");
        TurnToAngle turnToAngle = new TurnToAngle(drivetrain, res.getDriveAngleDeg());

        return Commands.sequence(
            turnToAngle, 
            new GoToAngle(arm, angle),
            Commands.runOnce(() -> {
              drivetrain.swerveDrive(new ChassisSpeeds(), false);
            }),
            SetShooterSpeed.indexNote(intake),
            OuttakeToSpeaker.revAndIndex(intake, topSpeed, bottomSpeed), 
            OuttakeToSpeaker.shoot(intake)
        ).finallyDo(() -> {
          intake.stopShoot();
          intake.stopSuck(); 
        });
    }, Set.of(drivetrain, arm, intake)); 
  }
}
