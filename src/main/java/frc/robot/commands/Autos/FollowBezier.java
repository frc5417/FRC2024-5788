// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Bezier;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Bezier.BezierFunction;

public class FollowBezier extends Command {
  DriveBase m_drivebase;
  BezierFunction bezierFunction;
  double time = 0.0;
  double steps;
  Field2d field = new Field2d();
  /** Creates a new FollowBezier. */
  public FollowBezier(DriveBase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivebase = drivebase;
    // bezierFunction = Bezier.computeBezier(null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, 0));
  }

  public void setPath(Pose2d[] points, double stepsToComplete) {
    steps = stepsToComplete;
    bezierFunction = Bezier.computeBezier(points);
    time = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d computedPose = bezierFunction.apply(time);
    time += 1/steps;
    field.setRobotPose(computedPose);
    SmartDashboard.putData(field);
    SmartDashboard.updateValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
