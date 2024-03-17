// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.PhotonSubsystem;

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
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Wrist m_wrist;
  private final PhotonSubsystem m_photonsubsystem;

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;

  double wristPos = 0.0;

  double manipulatorPosition = 0;

  public TeleopDrive(DriveBase driveBase, Elevator elevator, Intake intake, Shooter shooter, Wrist wrist, PhotonSubsystem photonsubsystem) {
    m_driveBase = driveBase;
    m_elevator = elevator;
    m_intake = intake;
    m_shooter = shooter;
    m_wrist = wrist;
    m_photonsubsystem = photonsubsystem;
  }

  @Override
  public void initialize() {
    RobotContainer.setLEDsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double xVel = (RobotContainer.getDriverLeftJoyX() * 0.45) + (prev_xVel * 0.55); 
    double yVel = (RobotContainer.getDriverLeftJoyY() * 0.45) + (prev_yVel * 0.55); 
    double omega = (RobotContainer.getDriverRightJoyX() * 0.45) + (prev_omega * 0.55);

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;

    SmartDashboard.putNumber("X-Vel Input", xVel);
    SmartDashboard.putNumber("Y-Vel Input", yVel);
    SmartDashboard.putNumber("Omega Vel Input", omega);
    
    m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(xVel, yVel, omega));

    m_elevator.setElevatorPower(RobotContainer.getElevatorLeftJoystick());

    m_intake.setIntakePower(RobotContainer.getIntakeRightTrigger() - RobotContainer.getIntakeLeftTrigger());
    
    m_shooter.setShooterPower(RobotContainer.getShooterRightTrigger());

    m_shooter.setShooterIntake(RobotContainer.getShooterIntakeSpeed() + RobotContainer.getShooterIntakeReverseSpeed());

    
    wristPos += -RobotContainer.getWristRightJoystick() * 0.01;
    wristPos =MathUtil.clamp(wristPos, 0, 0.7);

    m_wrist.setWristPos(wristPos);

    if (RobotContainer.getManipulatorBBool()) {
      wristPos = 0.0238;
    }

    if (RobotContainer.getManipulatorABool()) {
      wristPos = 0.0523;
    }

    if (RobotContainer.getManipulatorYBool()) {
      wristPos = 0.0015;
    }

    m_photonsubsystem.updatePose();

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
