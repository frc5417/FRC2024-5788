// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoControllers;

import java.lang.reflect.Field;

import com.fasterxml.jackson.databind.node.POJONode;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.EnumKeySerializer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Bezier;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.TargetStateRun;
import frc.robot.subsystems.Bezier.BezierFunction;

public class FollowBezier extends Command {
  TargetStateRun m_targetstaterun;
  BezierFunction bezierFunction;
  double time = 0.0;
  double steps;
  Field2d field = new Field2d();
  Pose2d finalPose;
  boolean terminate = false;
  /** Creates a new FollowBezier. */
  public FollowBezier(TargetStateRun targetstaterun) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_targetstaterun = targetstaterun;
    finalPose = m_targetstaterun.m_drivebase.getCurrentPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetstaterun.m_drivebase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, 0));
  }

  public void setPath(Pose2d[] points, double stepsToComplete) {
    steps = stepsToComplete;
    bezierFunction = Bezier.computeBezier(points);
    for (int i=0; i < 10; i++) {
      System.out.printf("%s, %s\n", i/10.0, bezierFunction.apply(i/10.0));
    }
    time = 0.0;
    finalPose = points[points.length-1];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentX = m_targetstaterun.m_drivebase.getCurrentPose().getX();
    double currentY = m_targetstaterun.m_drivebase.getCurrentPose().getY();
    double currentTheta = m_targetstaterun.m_drivebase.getCurrentPose().getRotation().getDegrees();
    // if (Math.abs(currentX-finalPose.getX())>Constants.Auton.poseTolerance || Math.abs(currentY-finalPose.getY())>Constants.Auton.poseTolerance || Math.abs(currentTheta-finalPose.getRotation().getDegrees())>Constants.Auton.thetaTolerance) {
    if (finalPose != bezierFunction.apply(time)) {
      Pose2d computedPose = bezierFunction.apply(time);
      m_targetstaterun.setTarget(computedPose);
      time += 1/steps;
      Pose2d invertedPose = new Pose2d(computedPose.getY(), computedPose.getX(), computedPose.getRotation());
      field.setRobotPose(invertedPose);
    } else {
      terminate = true;
    }
    SmartDashboard.putData(field);
    SmartDashboard.putString(getSubsystem(), "Running Bezier");
    SmartDashboard.updateValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_targetstaterun.m_drivebase.setDriveSpeed(new ChassisSpeeds());
    SmartDashboard.putString(getSubsystem(), "BezierEnded");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
