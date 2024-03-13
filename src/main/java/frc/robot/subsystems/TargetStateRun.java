// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TargetStateRun extends SubsystemBase {
  public DriveBase m_drivebase;

  PIDController x_pid = Constants.Auton.X_Pos;
  PIDController y_pid = Constants.Auton.Y_Pos;
  PIDController omega_pid = Constants.Auton.Theta_Pos;

  Pose2d targetPose;

  /** Creates a new TargetStateRun. */
  public TargetStateRun(DriveBase drivebase) {
    m_drivebase = drivebase;
    targetPose = drivebase.getCurrentPose();
  }

  public void setTarget(Pose2d target) {
    targetPose = target;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isAutonomousEnabled()) {
      double currentX = m_drivebase.getCurrentPose().getX();
      double currentY = m_drivebase.getCurrentPose().getY();
      double currentTheta = m_drivebase.getCurrentPose().getRotation().getDegrees();

      x_pid.setSetpoint(targetPose.getX());
      y_pid.setSetpoint(targetPose.getY());
      omega_pid.setSetpoint(targetPose.getRotation().getDegrees());
      
      if (Math.abs(currentX-targetPose.getX())>Constants.Auton.poseTolerance || Math.abs(currentY-targetPose.getY())>Constants.Auton.poseTolerance || Math.abs(currentTheta-targetPose.getRotation().getDegrees())>Constants.Auton.thetaTolerance) {
        m_drivebase.setDriveSpeed(new ChassisSpeeds(MathUtil.clamp(x_pid.calculate(currentX), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), MathUtil.clamp(y_pid.calculate(currentY), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), MathUtil.clamp(omega_pid.calculate(currentTheta), -Constants.Auton.speedClamp, Constants.Auton.speedClamp)));
        // SmartDashboard.putNumber("X_SPEED", MathUtil.clamp(x_pid.calculate(currentX, targetPose.getX()), -Constants.Auton.speedClamp, Constants.Auton.speedClamp));
        // SmartDashboard.putNumber("Y_SPEED", MathUtil.clamp(y_pid.calculate(currentY, targetPose.getY()), -Constants.Auton.speedClamp, Constants.Auton.speedClamp));
        // SmartDashboard.putNumber("Omega_SPEED", MathUtil.clamp(omega_pid.calculate(currentTheta, targetPose.getRotation().getDegrees()), -Constants.Auton.speedClamp, Constants.Auton.speedClamp));
        // double[] hehe = {x_pid.getPositionError(), y_pid.getPositionError(), omega_pid.getPositionError()};
        // SmartDashboard.putNumberArray("Error", hehe);
      } else {
        m_drivebase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, 0));
      }
      // SmartDashboard.updateValues();
    }
  }
}
