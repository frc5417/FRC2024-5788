// Copyright (c) FIRST and other WPILib contributors. test
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;

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
    public static final Double[] motorDegrees = {165.2324, 315.5274, 298.03716, 81.03528};
    public static final Double degTolerance = 0.75;
    public static final boolean[] invertedMotors = {true, true, true, true};

  }

  public static class ManipulatorConstants {}
  
  public static class Swerve {
    public static final Double angularPercentage = -0.1;
    public static final Double XPercentage = -0.3;
    public static final Double YPercentage = -0.3;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    public static final double maxVelocity = 3.8; // m/s
    public static final double maxAngularVelocity = 10; //rad/sec
    public static final double maxModuleSpeed = 0.3;
    public static final boolean shouldFlipAuto = true;
  }

  public static class DriveBaseConstants {
    public static final double driveBaseRadius = (Double) 0.51 * Math.sqrt(2); 
  }

  public static class Auton {
    public static final String[] paths = {"rotateInPlace, moveForward, PathPlannerTest"};
  }

  // public static class VisionConstants {
  //   public static final Transform3d robotToCam =
  //           new Transform3d(
  //                  new Translation3d(0.5, 0.0, 0.5),
  //                   new Rotation3d(
  //                           0, 0,
  //                           0)); 

  //   // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  //   public static final String cameraName = "OV5647";
  //   public static final double maxDistanceAway = 2.0;
  //   public static final double forwardKP = 0.1;
  //   public static final double forwardToAngleRatio = 0.5;
    
  //   public static final double CAMERA_HEIGHT_METERS = 0.72;
  //   public static final double TARGET_HEIGHT_METERS = 0;
  //   public static final double CAMERA_PITCH_RADIANS = 0;
  // }

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);
  } 
}
