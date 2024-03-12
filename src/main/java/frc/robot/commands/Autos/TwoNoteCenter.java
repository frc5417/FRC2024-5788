// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.TargetStateRun;

public class TwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new TwoNoteCenter. */
  public TwoNoteCenter(TargetStateRun targetStateRun) {
    // Use addRequirements() here to declare subsystem dependencies.
    FollowBezier forward = new FollowBezier(targetStateRun);
    Pose2d[] path = {new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 4, Rotation2d.fromDegrees(90)), new Pose2d(5, 5, Rotation2d.fromDegrees(180)), new Pose2d(10, 3, Rotation2d.fromDegrees(0))};
    forward.setPath(path, 200);

    FollowBezier backward = new FollowBezier(targetStateRun);
    Pose2d[] path2 = {new Pose2d(10, 3, Rotation2d.fromDegrees(0)), new Pose2d(0, 4, Rotation2d.fromDegrees(90))};
    backward.setPath(path2, 200);

    addCommands(forward, backward);
  }
}
