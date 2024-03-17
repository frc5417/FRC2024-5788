// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.TargetStateRun;
import frc.robot.subsystems.A_Star.A_Star;
import frc.robot.subsystems.A_Star.Node;

public class TwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new TwoNoteCenter. */
  public TwoNoteCenter(TargetStateRun targetStateRun, Pose2d startingPose) {
    // A_Star pathSolver = new A_Star(Constants.Auton.robot_size, Constants.Auton.field_size);
    
    Pose2d[] path1 = A_Star.nodeListToPoses(A_Star.compute(new Node(0.66, 0.66, false), new Node(4, 9, false)));
    Pose2d[] path2 = A_Star.nodeListToPoses(A_Star.compute(new Node(4, 9, false), new Node(7, 14, false)));
    A_Star.setEndRotation(path1, Rotation2d.fromDegrees(90));
    path2[0] = new Pose2d(path2[0].getX(), path2[0].getY(), Rotation2d.fromDegrees(90));

    FollowBezier temp1 = new FollowBezier(targetStateRun);
    FollowBezier temp2 = new FollowBezier(targetStateRun);
    // System.out.println(hehehehe);
    temp1.setPath(new Pose2d[] {new Pose2d(0, 0, new Rotation2d(0.0)), new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90)), new Pose2d(0.0, 4.0, new Rotation2d())}, 100);
    targetStateRun.m_drivebase.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));
    // temp2.setPath(path2, 100);

    // for (Pose2d p : hehehehe) {
    //   System.out.printf("[%s, %s], ", p.getX(), p.getY());
    // }
    addCommands(temp1);
  }
}
