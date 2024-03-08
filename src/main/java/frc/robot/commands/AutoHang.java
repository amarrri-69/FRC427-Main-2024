package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.util.GeometryUtils;

public class AutoHang {


    // basically tbe domainn and range for x and y values of the command
    public static boolean isPoseInRectangle(Pose2d position, Pose2d topLeft, Pose2d bottomRight, Pose2d topRight, Pose2d bottomLeft) {
        boolean betweenX = position.getX() > topLeft.getX() && position.getX() < bottomRight.getX();
        boolean betweenY = position.getY() < topLeft.getY() && position.getY() < bottomRight.getY();

        if (betweenX && betweenY)  {
            return true;
        }
        else {
            return false;
        }
    }

    // write a method that returns an angle based on the position it's in 
    // if the robot is in allaicne red but the robot is in a blue thingy, then just return 0 or somtehing
    public static double getAngle(Pose2d robotPose) {
        Alliance alliance = getTargetPose();

        if (alliance == Alliance.Blue) {
            if (isPoseInRectangle(robotPose, 
                Constants.AutoHang.blueBottomLeft1,
                Constants.AutoHang.blueBottomRight1,
                Constants.AutoHang.blueTopRight1,
                Constants.AutoHang.blueTopLeft1
            )) {
                return 120;
            }
            else if (isPoseInRectangle(robotPose, 
                Constants.AutoHang.blueBottomLeft2,
                Constants.AutoHang.blueBottomRight2,
                Constants.AutoHang.blueTopRight2,
                Constants.AutoHang.blueTopLeft2
            )) {
                return 0;
            }
            else if (isPoseInRectangle(robotPose, 
                Constants.AutoHang.blueBottomLeft3,
                Constants.AutoHang.blueBottomRight3,
                Constants.AutoHang.blueTopRight3,
                Constants.AutoHang.blueTopLeft3
            )) {
                return -120;
            }
        }
        if (alliance == Alliance.Red) {
            if (isPoseInRectangle(robotPose, 
                Constants.AutoHang.redBottomLeft1,
                Constants.AutoHang.redBottomRight1,
                Constants.AutoHang.redTopRight1,
                Constants.AutoHang.redTopLeft1
            )) {
                return 60;
            }
            else if (isPoseInRectangle(robotPose, 
                Constants.AutoHang.redBottomLeft2,
                Constants.AutoHang.redBottomRight2,
                Constants.AutoHang.redTopRight2,
                Constants.AutoHang.redTopLeft2
            )) {
                return -180;
            }
            else if (GeometryUtils.isPoseInRectangle(robotPose, 
                Constants.AutoHang.redBottomLeft3,
                Constants.AutoHang.redBottomRight3,
                Constants.AutoHang.redTopRight3,
                Constants.AutoHang.redTopLeft3
            )) {
                return -60;
            }
        }

        return 0; 
    }

    public static Alliance getTargetPose() {
        Optional<Alliance> optAlliance = DriverStation.getAlliance();

        if (optAlliance.isEmpty()) return null;

        Alliance alliance = optAlliance.get(); 
        return alliance; 
    }
}
