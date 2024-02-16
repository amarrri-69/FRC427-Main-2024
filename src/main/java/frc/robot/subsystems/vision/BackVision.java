package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drivetrain.Drivetrain;

// Figures out the position of the robot based on april tags
public class BackVision extends SubsystemBase{
    
    public static BackVision instance; 
    //  = new BackVision(Drivetrain.getInstance());

    private Drivetrain drivetrain;
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private Transform3d latestPoseResult;

    private BackVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.camera = new PhotonCamera("backPhotonCamera");
    }

    public void periodic() {
        this.latestResult = this.camera.getLatestResult();
        this.latestPoseResult = this.latestResult.getMultiTagResult().estimatedPose.best;
        // SmartDashboard.putNumber("RobotPositionX", networkTable.);
    }

    public static BackVision getInstance() {
        return instance;
    }

    public PhotonPipelineResult getLatestVisionResult() {
        return latestResult;
    }

    // Calculates the stand deviation of the distance of the object
    private Matrix<N3, N1> calculateVisionStdDevs() {
        double distance = getDistanceToAprilTag();
        var translationStdDev = Constants.Vision.kTranslationStdDevCoefficient * Math.pow(distance, 2);
        var rotationStdDev = Constants.Vision.kRotationStdDevCoefficient * Math.pow(distance, 2);
        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    // Gets the id of the closest april tag
    private int getBestAprilTagID() {
        return latestResult.getBestTarget().getFiducialId();
    }
    
    // Returns the pose3d of the closest april tag
    private Pose3d getAprilTagPos(int aprilTagID) {
        return Constants.Vision.kAprilTagFieldLayout.getTagPose(aprilTagID).orElse(null);
    }

    // Gets the distance to the closest april tag by finding the 3-dimensional norm
    private double getDistanceToAprilTag() {
        Pose3d aprilTagPose = getAprilTagPos(getBestAprilTagID());
        if (aprilTagPose == null) return 1000;
        return getDistanceBetweenPose3d(aprilTagPose, getCurrentPose3d());
    }

    // Returns the distance between two pose3d objects in three dimensions
    private double getDistanceBetweenPose3d(Pose3d firstPose, Pose3d secondPose) {
        double xDifferenceSquared = Math.pow(secondPose.getX() - firstPose.getX(), 2);
        double yDifferenceSquared = Math.pow(secondPose.getY() - firstPose.getY(), 2);
        double zDifferenceSquared = Math.pow(secondPose.getZ() - firstPose.getZ(), 2);
        return Math.sqrt(xDifferenceSquared + yDifferenceSquared + zDifferenceSquared);
    }

     // Returns the distance between two pose3d objects in three dimensions
    private double getDistanceBetweenPose2d(Pose2d firstPose, Pose2d secondPose) {
        double xDifferenceSquared = Math.pow(secondPose.getX() - firstPose.getX(), 2);
        double yDifferenceSquared = Math.pow(secondPose.getY() - firstPose.getY(), 2);
        return Math.sqrt(xDifferenceSquared + yDifferenceSquared);
    }

    // Returns the rotation3d using values stored in the network table
    private Rotation3d getRotation3d() {
        return this.latestPoseResult.getRotation();
    }

    // Returns the rotation3d using values stored in the network table
    private Pose3d getCurrentPose3d() {
        return new Pose3d().transformBy(this.latestPoseResult); // TODO: does this work?
    }

    // Adds vision measurements to the drivetrain if they are within the field
    public void addVisionFromDrivetrain() {
        if (!isPoseValid()) return; 
        drivetrain.addVisionPoseEstimate(getCurrentPose3d(), getDistanceToAprilTag(), Timer.getFPGATimestamp() - (this.latestResult.getLatencyMillis() / 1000), calculateVisionStdDevs());
    }

    public boolean isEstimateClose() {
        Pose2d currentPose2d = getCurrentPose3d().toPose2d();
        return (getDistanceBetweenPose2d(currentPose2d, drivetrain.getPose()) < Constants.Vision.kMaxAccuracyRange);
    }

    // Sees if the robot's position as given by the limelight is within the field
    public boolean isPoseValid() {
        boolean isInLowerXLimit = -2.5 <= getCurrentPose3d().getX();
        boolean isInUpperXLimit = getCurrentPose3d().getX() <= Constants.Vision.kAprilTagFieldLayout.getFieldLength() + 2.5;
        boolean isInLowerYLimit = -2.5 <= getCurrentPose3d().getY();
        boolean isInUpperYLimit = getCurrentPose3d().getY() <= Constants.Vision.kAprilTagFieldLayout.getFieldWidth() + 2.5;
        boolean isInLowerZLimit = 0 <= getCurrentPose3d().getZ();
        boolean isInUpperZLimit = getCurrentPose3d().getZ() <= Constants.Vision.limelightZHeight + 0.5;
        return isInLowerXLimit && isInUpperXLimit && isInLowerYLimit && isInUpperYLimit && isInLowerZLimit && isInUpperZLimit && isEstimateClose();
    }

}
