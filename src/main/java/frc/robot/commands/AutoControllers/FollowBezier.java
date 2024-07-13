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
import frc.robot.subsystems.Bezier;
import frc.robot.subsystems.Bezier.BezierFunction;
import frc.robot.subsystems.DriveBase;

public class FollowBezier extends Command {
  DriveBase m_driveBase;
  BezierFunction bezierFunction;
  double time = 0.0;
  double steps;
  Pose2d finalPose;
  boolean terminate = false;

  PIDController x_pid = Constants.Auton.X_Pos;
  PIDController y_pid = Constants.Auton.Y_Pos;
  PIDController omega_pid = Constants.Auton.Theta_Pos;

  /** Creates a new FollowBezier. */
  public FollowBezier(DriveBase driveBase, Pose2d[] path, int steps, boolean resetInit) {
    // Use addRequirements() here to declare subsystem dependencies.
    omega_pid.enableContinuousInput(-180, 180);
    m_driveBase = driveBase;
    if (resetInit) {
      m_driveBase.resetOdometry(path[0]);
    }
    finalPose = m_driveBase.getCurrentPose();
    setPath(path, steps);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveBase.setDriveSpeed(new ChassisSpeeds());
  }

  private void setPath(Pose2d[] points, double stepsToComplete) {
    steps = stepsToComplete;
    bezierFunction = Bezier.computeBezier(points);
    for (int i=0; i < 10; i++) {
      System.out.printf("%s, %s\n", i/10.0, bezierFunction.apply(i/10.0));
    }
    time = 0.0;
    finalPose = points[points.length-1];
  }

  void setPIDSetpoints(Pose2d targetPose) {
    x_pid.setSetpoint(targetPose.getX());
    y_pid.setSetpoint(targetPose.getY());
    // omega_pid.setSetpoint(getGoodAngle(targetPose.getRotation().getRadians()));
    omega_pid.setSetpoint(targetPose.getRotation().getDegrees());
  }

  double getGoodAngle(double thetaRad) {
    if (thetaRad < 0) {
      return thetaRad + 2 * Math.PI;
    }
    return thetaRad;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentX = m_driveBase.getCurrentPose().getX();
    double currentY = m_driveBase.getCurrentPose().getY();
    double currentTheta = m_driveBase.getCurrentPose().getRotation().getDegrees(); //getGoodAngle(m_driveBase.getCurrentPose().getRotation().getRadians());

    
    
    if (finalPose != bezierFunction.apply(time)) {
      // Pose2d computedPose = RobotContainer.WPI_to_Custom(bezierFunction.apply(time));
      setPIDSetpoints(bezierFunction.apply(time));
      // m_driveBase.resetOdometry(bezierFunction.apply(time));
      // Pose2d stupid = new Pose2d(computedPose.getX()*-1, computedPose.getY(), computedPose.getRotation());
      // m_targetstaterun.m_drivebase.resetOdometry(computedPose.times(-1));
      time += 1/steps;
    } 

    if (Math.abs(currentX-finalPose.getX())>Constants.Auton.poseTolerance || Math.abs(currentY-finalPose.getY())>Constants.Auton.poseTolerance || Math.abs(currentTheta-finalPose.getRotation().getDegrees())>Constants.Auton.thetaTolerance) {
      if (m_driveBase.isRed()) {
        m_driveBase.setRedAutoSpeed(new ChassisSpeeds(MathUtil.clamp(x_pid.calculate(currentX), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), MathUtil.clamp(y_pid.calculate(currentY), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), MathUtil.clamp(omega_pid.calculate(currentTheta), -Constants.Auton.speedRotClamp, Constants.Auton.speedRotClamp)));
      } else {
        m_driveBase.setBlueAutoSpeed(new ChassisSpeeds(MathUtil.clamp(x_pid.calculate(currentX), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), MathUtil.clamp(y_pid.calculate(currentY), -Constants.Auton.speedClamp, Constants.Auton.speedClamp), MathUtil.clamp(omega_pid.calculate(currentTheta), -Constants.Auton.speedRotClamp, Constants.Auton.speedRotClamp)));
      }
    } else {
      terminate = true;
      m_driveBase.setBlueAutoSpeed(new ChassisSpeeds());
    }

    

    // SmartDashboard.putNumber("OLDTHETA", stupid);
    // SmartDashboard.putNumber("CURRENT_THETA", currentTheta);
    // SmartDashboard.putNumber("OMEGASETPOINT", omega_pid.getSetpoint());

    // SmartDashboard.putData(field);
    // SmartDashboard.putString(getSubsystem(), "Running Bezier");
    // SmartDashboard.updateValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.setDriveSpeed(new ChassisSpeeds());
    // SmartDashboard.putString(getSubsystem(), "BezierEnded");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
