// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.LightsControl;

import java.io.Console;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.

  private final DriveBase m_driveBase;
  // private final Elevator m_elevator;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Wrist m_wrist;
  private final PhotonSubsystem m_photonsubsystem;
  private final LightsControl m_lightscontrol;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;
  int timer = 0;
  int lastTime = 0;

  double wristPos = 0.0;

  double manipulatorPosition = 0;

  public TeleopDrive(DriveBase driveBase, Intake intake, Shooter shooter, Wrist wrist, PhotonSubsystem photonsubsystem, LightsControl lightscontrol) {
    m_driveBase = driveBase;
    // m_elevator = elevator;
    m_intake = intake;
    m_shooter = shooter;
    m_wrist = wrist;
    m_photonsubsystem = photonsubsystem;
    m_lightscontrol = lightscontrol;

    // m_photonsubsystem.setDriverMode(true);

    // m_driveBase.resetOdometry(new Pose2d(1.5, 5.5, new Rotation2d()));
    // m_driveBase.resetOdometry(new Pose2d(15.5, 5.5, Rotation2d.fromDegrees(180)));
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double xVel = (RobotContainer.getDriverLeftJoyX() * 0.90) + (prev_xVel * 0.10); 
    double yVel = (RobotContainer.getDriverLeftJoyY() * 0.90) + (prev_yVel * 0.10); 
    double omega = (RobotContainer.getDriverRightJoyX() * 0.90) + (prev_omega * 0.10);

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;

    // SmartDashboard.putNumber("X-Vel Input", xVel);
    // SmartDashboard.putNumber("Y-Vel Input", yVel);
    // SmartDashboard.putNumber("Omega Vel Input", omega);
    
    if (RobotContainer.getDriveXBool()) {
      m_driveBase.X_MODE();
      
    } else {
      m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(xVel, yVel, omega));
    }

    // m_elevator.setElevatorPower(RobotContainer.getElevatorLeftJoystick());

    m_intake.setIntakePower(RobotContainer.getIntakeRightTrigger() - RobotContainer.getIntakeLeftTrigger());
    
    m_shooter.setShooterPower(RobotContainer.getShooterRightTrigger());

    m_shooter.setShooterIntake(RobotContainer.getShooterIntakeSpeed() + RobotContainer.getShooterIntakeReverseSpeed());

    
    wristPos += -RobotContainer.getWristRightJoystick() * 0.01;
    wristPos = MathUtil.clamp(wristPos, 0, 0.3);

    m_wrist.setWristPos(wristPos);

    if (RobotContainer.getManipulatorBBool()) { //shoot from subwoofer
      wristPos = Constants.Wrist.shoot_from_subwoofer;
    }

    if (RobotContainer.getManipulatorABool()) { //handoff
      wristPos = Constants.Wrist.handoff;

    }

    if (RobotContainer.getManipulatorYBool()) { //experimental amp
      wristPos = Constants.Wrist.amp;
      m_shooter.setDifferentialShooterPower(-0.45, -0.09);
    }

    if(RobotContainer.getManipulatorXBool()) { //photon shoot
      // wristPos = m_wrist.setWristDeg(m_photonsubsystem.getOptimalAngle());
      // SmartDashboard.putNumber("Ideal Deg Wrist", m_photonsubsystem.getOptimalAngle());
    }
    // m_lightscontrol.setLed(4);

    if (RobotContainer.getManipulatorRightBumperBool()) {
      wristPos = Constants.Wrist.passing;
      m_shooter.setShooterPower(-0.75);
    }

    timer += 1;

    if (m_intake.noteInIntake() == true){
      lastTime = timer;

      // m_lightscontrol.setLed(2);
    }

    if (timer == (lastTime+(3/0.02))) {
      // m_lightscontrol.setLed(0);
    }
    

    // m_driveBase.resetOdometry(RobotContainer.WPI_to_Custom(m_photonsubsystem.getEstimatedFieldPose()));
    // Pose2d computedPose = m_driveBase.getCurrentPose();
    
    // Pose2d invertedPose = RobotContainer.Custom_to_WPI(computedPose);

    // SmartDashboard.putNumber("X_Field", computedPose.getX());
    // SmartDashboard.putNumber("Y_Field",computedPose.getY());
    // SmartDashboard.putNumber("X_Field2", m_photonsubsystem.getEstimatedFieldPose().getX());
    // SmartDashboard.putNumber("Y_Field2",m_photonsubsystem.getEstimatedFieldPose().getY());
    // SmartDashboard.putNumber("Omega_Fielf", m_photonsubsystem.getEstimatedFieldPose().getRotation().getDegrees());
    // SmartDashboard.updateValues();



    // if (RobotContainer.getDriveBBool()) {
    //   m_driveBase.setSnapping(true);
    // } else {
    //   m_driveBase.setSnapping(false);
    //   m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(xVel, yVel, omega));
    // }
    // m_wrist.setWristPos(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.resetDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
