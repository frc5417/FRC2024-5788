// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoloTune extends SequentialCommandGroup {
  /** Creates a new RedSourceSide. */
  public HoloTune(DriveBase driveBase) {
    // Add your commands in the addCommands() call, e.g.
    Pose2d one = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    
    Pose2d left = new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(-90));
    Pose2d mid = new Pose2d(3, 0.0, Rotation2d.fromDegrees(-180));
    Pose2d right = new Pose2d(4.5, -2.0, Rotation2d.fromDegrees(-270));
    Pose2d endpose = new Pose2d(5.5, 0, Rotation2d.fromDegrees(-360));
    Pose2d left2 = new Pose2d(4.5, 1.5, Rotation2d.fromDegrees(-450));
    Pose2d mid2 = new Pose2d(3, 0.0, Rotation2d.fromDegrees(-540));
    Pose2d right2 = new Pose2d(1.5, -1.5, Rotation2d.fromDegrees(-63));

    Pose2d[] circlefirst = {one, left, mid, right, endpose};
    Pose2d[] circlesecond = {endpose, left2, mid2, right2};



    FollowBezier circleB = new FollowBezier(driveBase, circlefirst, 500, true);
    FollowBezier circleD = new FollowBezier(driveBase, circlesecond, 500, false);

    addCommands(
      circleB,
      circleD
      );
  }
}
