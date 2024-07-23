// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonLoader;
import frc.robot.commands.TeleopDrive;

public class Loggy extends SubsystemBase {
  /** Creates a new Loggy. */
  AHRS m_ahrs;
  DriveBase m_driveBase;
  Intake m_intake;
  Shooter m_shooter;
  Wrist m_wrist;
  PhotonSubsystem m_photonSubsystem;
  Bezier m_bezier;
  LightsControl m_lightsControl;
  AutonLoader m_autonLoader;
  TeleopDrive m_teleopDrive;

  ShuffleboardTab UserInputs = Shuffleboard.getTab("User Inputs");
  ShuffleboardTab Auton = Shuffleboard.getTab("Autonomous");
  ShuffleboardTab Teleop = Shuffleboard.getTab("Teleoperated");

  public Loggy(AHRS ahrs, DriveBase driveBase, Intake intake, Shooter shooter, Wrist wrist, PhotonSubsystem photonSubsystem, Bezier bezier, LightsControl lightsControl, AutonLoader autonLoader, TeleopDrive teleopDrive) {
    m_ahrs = ahrs;
    m_driveBase = driveBase;
    m_intake = intake;
    m_shooter = shooter;
    m_wrist = wrist;
    m_photonSubsystem = photonSubsystem;
    m_bezier = bezier;
    m_lightsControl = lightsControl;
    m_autonLoader = autonLoader;
    m_teleopDrive = teleopDrive;
    
    Auton.add(m_autonLoader.getChooser());
    Auton.add("Odometry", driveBase.field);
    Teleop.add(m_ahrs);
    Teleop.addDoubleArray("Supplied Chassis Speed", m_driveBase.chassisSpeed_supp);
    Teleop.addDouble("Intake Speed", m_intake.intakeSpeed_supp);
    Teleop.addDouble("Intake Amps", m_intake.intakeAmps_supp);
    Teleop.addDoubleArray("Shooter Speeds", m_shooter.shooterSpeed_supp);
    Teleop.addDouble("Indexer Speed", m_shooter.indexSpeed_supp);
    Teleop.add("Limelight", m_photonSubsystem.estFieldPose);
    Teleop.addDouble("Distance to Nearest Target", m_photonSubsystem.distanceToTarget_supp);
    Teleop.addDouble("Estimated Shoot Angle", m_photonSubsystem.estShootAngle_supp);

    Teleop.addDouble("Wrist", m_wrist.wristSupplier);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
