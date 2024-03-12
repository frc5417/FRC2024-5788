// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoControllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class SimpleLinear extends Command {
  /** Creates a new MoveToPos. */
  DriveBase m_drivebase;

  PIDController x_pid = Constants.Auton.X_Pos;
  PIDController y_pid = Constants.Auton.Y_Pos;
  PIDController omega_pid = Constants.Auton.Theta_Pos;

  Pose2d targetPoseDelta;

  double targetX, targetY;

  private boolean terminate = false; 

  public SimpleLinear(DriveBase driveBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = driveBase;
    targetPoseDelta = m_drivebase.getCurrentPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, 0));
  }


  public void setTarget(Pose2d poseDelta) {
    targetPoseDelta = poseDelta;
    double currentX = m_drivebase.getCurrentPose().getX();
    double currentY = m_drivebase.getCurrentPose().getY();
    targetX = currentX + targetPoseDelta.getX();
    targetY = currentY + targetPoseDelta.getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentX = m_drivebase.getCurrentPose().getX();
    double currentY = m_drivebase.getCurrentPose().getY();
    
    if (Math.abs(currentX-targetX)>Constants.Auton.poseTolerance || Math.abs(currentY-targetY)>Constants.Auton.poseTolerance) {
      m_drivebase.setDriveSpeed(new ChassisSpeeds(MathUtil.clamp(x_pid.calculate(currentX, targetX), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), MathUtil.clamp(y_pid.calculate(currentY, targetY), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), 0.0));
    } else {
      m_drivebase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, 0));
      terminate = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
