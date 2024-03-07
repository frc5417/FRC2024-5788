// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

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

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;

  double wristPos = 0.0;

  double manipulatorPosition = 0;

  public TeleopDrive(DriveBase driveBase, Elevator elevator, Intake intake, Shooter shooter, Wrist wrist) {
    m_driveBase = driveBase;
    m_elevator = elevator;
    m_intake = intake;
    m_shooter = shooter;
    m_wrist = wrist;
  }

  @Override
  public void initialize() {
    // m_driveBase.resetDrive();
    // Module.ModuleState[] temp = new Module.ModuleState[4];

    // for (int i = 0; i < 4; i++)
    //   temp[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));

    // m_driveBase.setHardStates(temp);
    RobotContainer.setLEDsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (counter++ <= 60)
    //   return;
    
    double xVel = (RobotContainer.getDriverLeftJoyX() * 0.45) + (prev_xVel * 0.55); 
    double yVel = (RobotContainer.getDriverLeftJoyY() * 0.45) + (prev_yVel * 0.55); 
    double omega = (RobotContainer.getDriverRightJoyX() * 0.45) + (prev_omega * 0.55);

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;

    SmartDashboard.putNumber("X-Vel Input", xVel);
    SmartDashboard.putNumber("Y-Vel Input", yVel);
    SmartDashboard.putNumber("Omega Vel Input", omega);
    
    // m_driveBase.setDriveSpeed(new ChassisSpeeds(xVel * Constants.Swerve.XPercentage, yVel * Constants.Swerve.YPercentage, omega * Constants.Swerve.angularPercentage));
    m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(xVel, yVel, omega));
    // m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0.5, yVel, omega));
    // m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(1, 1, 0));
    
    // m_manipulator.setWristSpeed(RobotContainer.getManipulatorRightJoyY());
    m_elevator.setElevatorPower(RobotContainer.getElevatorLeftJoystick());

    m_intake.setIntakePower(RobotContainer.getIntakeRightTrigger());
    
    m_shooter.setShooterPower(RobotContainer.getShooterLeftTrigger() - RobotContainer.getShooterRightTrigger());

    m_shooter.setShooterIntake(RobotContainer.getShooterIntakeSpeed());

    
    wristPos += -RobotContainer.getWristRightJoystick() * 0.001;
    wristPos =MathUtil.clamp(wristPos, 0, 0.7);

    m_wrist.wristSpinny(-RobotContainer.getWristRightJoystick());
    // m_wrist.setWristPos(0.15);
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
