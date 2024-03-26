// Copyright (c) FIRST and other WPILib contributors. test
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverPort = 0;
    public static final int kManipulatorPort = 1;
    public static final boolean fieldCentric = true; //FRONT IS THE SIDE OPPOSITE TO BATTERY
    public static final double joystickDeadband = 0.1; //HAS TO BE TUNED A BIT
  }

  public static class ModuleConstants {
    // 0 indexing
    public static final Integer[] driveMotorIDS = {11, 21, 31, 41}; 
    public static final Integer[] angleMotorIDS = {12, 22, 32, 42};
    public static final Integer[] CANCoderID = {13, 23, 33, 43};
    public static final Double[] motorDegrees = {167.1678, 127.52928, 39.55068, 315.96696};
    public static final Double degTolerance = 0.75;
    public static final boolean[] invertedMotors = {true, true, false, false};

  }

  public static class ManipulatorConstants {
    public static final int elevatorMaster = 61;
    public static final int elevatorSlave = 62;
    public static final int intake = 51;
    public static final int shooterMaster = 52;
    public static final int shooterSlave = 53;
    public static final int shooterIntake = 54;
    public static final int wrist = 55;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double wristTolerance = 0.02;
    public static final double shooterNominalAngle = 67;
    public static final double intakeCurrentLimit = 7;
  }
  
  public static class Swerve {
    public static final Double angularPercentage = -0.7;
    public static final Double XPercentage = -1.0;
    public static final Double YPercentage = -1.0;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    public static final double maxVelocity = 3.8; // m/s
    public static final double maxAngularVelocity = 10; //rad/sec
    public static final double maxModuleSpeed = 0.3;
    public static final boolean blueFlipState = false;
  }

  public static class DriveBaseConstants {
    public static final double driveBaseRadius = (Double) 0.51 * Math.sqrt(2); 
  }

  public static class Auton {
    public static final PIDController X_Pos = new PIDController(1.3, 0, 0);
    public static final PIDController Y_Pos = new PIDController(1.3, 0, 0);
    public static final PIDController Theta_Pos = new PIDController(0.0, 0.0, 0.0); //0.5 p 0.15 0.035
    public static final double speedClamp = 0.4;
    public static final double speedRotClamp = 0.3;
    public static final double poseTolerance = 0.1;
    public static final double thetaTolerance = 5; //degrees
    public static final Double[] robot_size = {0.66, 0.66};
    public static final Double[] field_size = {8.2, 16.0};
    public static final Double[] BlueObstacle_TopLeft = {2.5, 6.0};
    public static final Double[] BlueObstacle_BottomRight = {6.0,2.5};
    public static final Double[] RedObstacle_TopLeft = {2.5, 13.5};
    public static final Double[] RedObstacle_BottomRight = {6.0, 10.0};
    
  }

  public static class VisionConstants {
    // public static final Transform3d robotToCam =
    //         new Transform3d(
    //                new Translation3d(0.2794, 0.1524, 0.254),
    //                 new Rotation3d(
    //                         0, 
    //                         (66*(Math.PI/180)),
    //                         0)); 

    // camera is mounted facing backward, x is how far backward, y is how far left or right, z is how far high
    // CENTER AT THE CENTER OF THE DRIVETRAIN
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(-0.229, 0.254, 0.514), new Rotation3d(0,(75.0*(Math.PI/180.0)),(180.0*(Math.PI/180.0)))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  //   // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final String cameraName = "limelight";
    public static final double maxDistanceAway = 2.0;
    public static final double forwardKP = 0.1;
    public static final double forwardToAngleRatio = 0.5;
    
    public static final double CAMERA_HEIGHT_METERS = 1;
    public static final double TARGET_HEIGHT_METERS = 1.5;
    public static final double CAMERA_PITCH_RADIANS = 0;
  }

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);
    public static final double speakerHeightOptimal = 2.0-0.2;
    
    public static final double speakerExtensionOptimal = 0.15;
    public static final double pivotHeight = 0.381; //FIX THIS BUFFOON

    public static final Pose2d blueCenterNotePose = new Pose2d(3.5, 5.5, new Rotation2d());
    public static final Pose2d redCenterNotePose = new Pose2d(14.5, 5.5, Rotation2d.fromDegrees(-180));
  } 
}
