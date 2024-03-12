// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Bindings of the PathFlow Project by Krishna Shah (@DragonflyRobotics) https://github.com/DragonflyRobotics/PathFlow
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.Math;

public class Bezier extends SubsystemBase {
  /** Creates a new Bezier. */
  public Bezier() {}

  private static int factorial(int n){
    if (n<=1) {
      return 1;
    } else {
      return n * factorial(n-1);
    }
  }

  private static int nCr(int n, int k) {
      return factorial(n)/(factorial(k)*factorial(n-k));
  }
  public static BezierFunction computeBezier(Pose2d[] points) {
      int n = points.length - 1;
      return t -> {
          Pose2d pose = new Pose2d(0.0, 0.0, new Rotation2d());
          if (t <= 0.0) {
              return points[0];
          } else if (t >= 1.0) {
              return points[points.length - 1];
          } else {
              for (int i=0; i<points.length; i++) {
                  double X = nCr(n, i) * Math.pow(1-t, n-i) * Math.pow(t, i) * points[i].getX();
                  double Y = nCr(n, i) * Math.pow(1-t, n-i) * Math.pow(t, i) * points[i].getY();
                  double theta = nCr(n, i) * Math.pow(1-t, n-i) * Math.pow(t, i) * points[i].getRotation().getDegrees();
                  pose = new Pose2d(pose.getX()+X, pose.getY()+Y, Rotation2d.fromDegrees(pose.getRotation().getDegrees()+theta));
              }
              return pose;
          }
      };
  }

  @FunctionalInterface
  public interface BezierFunction {
      Pose2d apply(double t);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
