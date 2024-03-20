// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;
import java.lang.Math;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import frc.robot.Constants;

public class PhotonSubsystem extends SubsystemBase {
    /** Creates a new PhotonSubsystem. */
    public static PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
    public static PhotonPoseEstimator photonPoseEstimator;
    private int cnt = 0;
    private double[][] cameraInfo = new double[9][4]; // 1 indexed
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public PhotonSubsystem() {
        // photonCameraWrapper();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // updatePose();
    }

    // public PhotonCamera getCamera() {
    //     return photonCamera;
    // }

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

    // public void updatePose() {
    //     var result = photonCamera.getLatestResult();
    //     if(result.hasTargets()) {
    //         for (PhotonTrackedTarget target : result.getTargets()) {
    //             cameraInfo[target.getFiducialId()][0] = target.getBestCameraToTarget().getX();
    //             cameraInfo[target.getFiducialId()][1] = target.getBestCameraToTarget().getY();
    //             cameraInfo[target.getFiducialId()][2] = target.getBestCameraToTarget().getZ();
    //             // cameraInfo[target.getFiducialId()][3] = target.getPitch();
                
    //             // if((cnt++%10) == 0) {
    //             // System.out.print("FID: ");
    //             // System.out.print(target.getFiducialId());
    //             // System.out.print(", ");
    //             // System.out.print("X: ");
    //             // System.out.print(cameraInfo[target.getFiducialId()][0]);
    //             // System.out.print(", ");
    //             // System.out.print("Y: ");
    //             // System.out.print(cameraInfo[target.getFiducialId()][1]);
    //             // System.out.print(", ");
    //             // System.out.print("Z: ");
    //             // System.out.print(cameraInfo[target.getFiducialId()][2]);
    //             // System.out.print(", ");
    //             // System.out.print("Yaw: ");
    //             // System.out.print(cameraInfo[target.getFiducialId()][3]);
    //             // System.out.println("");
    //             SmartDashboard.putNumber("FID", target.getFiducialId());
    //             SmartDashboard.putNumber("X", target.getBestCameraToTarget().getX());
    //             SmartDashboard.putNumber("Y", target.getBestCameraToTarget().getY());
    //             SmartDashboard.putNumber("Z", target.getBestCameraToTarget().getZ());
    //             SmartDashboard.putNumber("Pitch", target.getPitch());
    //             // SmartDashboard.putNumber("Pitch", target.getPitch());

    //             SmartDashboard.updateValues();

    //             // }
    //         }
            
    //         // System.out.println(result.getTargets().get(0).getrFiducialId());
    //     }
    // }

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
                if ((target.getFiducialId() == 8) || (target.getFiducialId() == 4)) { //FIX THE 8 APRILTAG TO 7 IN ACTUAL FIELD
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