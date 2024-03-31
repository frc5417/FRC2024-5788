// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonSubsystem extends SubsystemBase {
    /** Creates a new PhotonSubsystem. */
    public static PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
    public static PhotonPoseEstimator photonPoseEstimator;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    

    public PhotonSubsystem() {
        // photonCameraWrapper();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // updatePose();
    }

    public double getYaw() {
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        else{
            return 0;
        }
    }

    public double getDistance() {
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.CAMERA_HEIGHT_METERS,
                    Constants.VisionConstants.TARGET_HEIGHT_METERS,
                    Constants.VisionConstants.CAMERA_PITCH_RADIANS,
                    Math.toRadians(result.getBestTarget().getPitch()));
        }
        else{
            return 0;
        }
    }

    public Pose3d getEstimatedGlobalPose() {
        Pose3d robotPose = new Pose3d();
        var result = photonCamera.getLatestResult();
        
        if(result.hasTargets()) {
            var target = result.getBestTarget();
            Optional<Pose3d> optionalTagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            // if (optionalTagPose.isPresent()) {}
            Pose3d tagPose = optionalTagPose.get();
            Pose3d estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose, Constants.VisionConstants.robotToCam);
            robotPose = estimatedPose;
        }
        return robotPose;
    }

    public Pose2d getEstimatedFieldPose() {
        Pose2d robotPose = new Pose2d();

        Pose3d estimatedPose = getEstimatedGlobalPose();
        robotPose = new Pose2d(estimatedPose.getX(), estimatedPose.getY(), new Rotation2d(estimatedPose.getRotation().getAngle()));
        return robotPose;
    }

    public double getOptimalAngle() {
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if ((target.getFiducialId() == 7) || (target.getFiducialId() == 4)) { //FIX THE 8 APRILTAG TO 7 IN ACTUAL FIELD
                    double distance = Math.sqrt((target.getBestCameraToTarget().getX()*target.getBestCameraToTarget().getX())+(target.getBestCameraToTarget().getY()*target.getBestCameraToTarget().getY()));
                    return (180.0/Math.PI)*Math.atan((Constants.FieldConstants.speakerHeightOptimal - Constants.FieldConstants.pivotHeight)/(distance - Constants.FieldConstants.speakerExtensionOptimal));
                }
            }
        }
        return Constants.ManipulatorConstants.shooterNominalAngle;
    }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation. Assumes a planar field and the robot is always firmly on the ground
   */
//   public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
//       photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//       return photonPoseEstimator.update();
//   }

}