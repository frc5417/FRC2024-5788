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
  public TwoNoteCenter(TargetStateRun targetStateRun) {
    A_Star pathSolver = new A_Star(Constants.Auton.robot_size, Constants.Auton.field_size);
    pathSolver.rectangularObstacle(new Double[] {2.5, 6.0}, new Double[] {6.0,2.5});
    Pose2d[] hehehehe = pathSolver.nodeListToPoses(pathSolver.compute(new Node(2, 2, false), new Node(7, 7, false)));
    pathSolver.setEndRotation(hehehehe, Rotation2d.fromDegrees(90));
    FollowBezier temp = new FollowBezier(targetStateRun);
    System.out.println(hehehehe);
    temp.setPath(hehehehe, 100);
    for (Pose2d p : hehehehe) {
      System.out.printf("[%s, %s], ", p.getX(), p.getY());
    }
    addCommands(temp);
  }
}
