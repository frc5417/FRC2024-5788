// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.ArrayList;
import java.util.List;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.TargetStateRun;
import frc.robot.subsystems.A_Star.A_Star;
import frc.robot.subsystems.A_Star.Node;

public class TwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new TwoNoteCenter. */
  public TwoNoteCenter(TargetStateRun targetStateRun) {
    // Use addRequirements() here to declare subsystem dependencies.
    List<FollowBezier> beziers = new ArrayList<FollowBezier>();
    // FollowBezier forward = new FollowBezier(targetStateRun);
    A_Star pathSolver = new A_Star(Constants.Auton.robot_size, Constants.Auton.field_size);
    pathSolver.rectangularObstacle(new Double[] {2.5, 6.0}, new Double[] {6.0,2.5});
    Pose2d[] hehehehe = pathSolver.nodeListToPoses(pathSolver.compute(new Node(2, 2, false), new Node(7, 7, false)));
    FollowBezier temp = new FollowBezier(targetStateRun);
    System.out.println(hehehehe);
    temp.setPath(hehehehe, 100);
    for (Pose2d p : hehehehe) {
      System.out.printf("[%s, %s], ", p.getX(), p.getY());
    }
    addCommands(temp);
    // for (int i=0; i < hehehehe.length-2; i++) {
    //   FollowBezier temp = new FollowBezier(targetStateRun);
    //   temp.setPath(new Pose2d[] {hehehehe[i], hehehehe[i+1], hehehehe[i+2]}, 200);
    //   beziers.add(temp);
    //   addCommands(temp);
    // }
    // Pose2d[] path = {new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 4, Rotation2d.fromDegrees(90)), new Pose2d(5, 5, Rotation2d.fromDegrees(180)), new Pose2d(10, 3, Rotation2d.fromDegrees(0))};
    // forward.setPath(hehehehe, 500);

    // FollowBezier backward = new FollowBezier(targetStateRun);
    // Pose2d[] path2 = {new Pose2d(10, 3, Rotation2d.fromDegrees(0)), new Pose2d(0, 4, Rotation2d.fromDegrees(90))};
    // backward.setPath(path2, 200);

    // addCommands(bezier);

  }
}
