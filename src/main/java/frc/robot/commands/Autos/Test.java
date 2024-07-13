// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends SequentialCommandGroup {
  Pose2d startPose = new Pose2d(0.73, 4.72, Rotation2d.fromDegrees(-60));
  Pose2d startPoseY = new Pose2d(3.0, 4.09, Rotation2d.fromDegrees(0));
  Pose2d note1 = new Pose2d(1.7, 4.09, Rotation2d.fromDegrees(0));

  Pose2d[] path1 = {startPose, note1};
  Pose2d[] path2 = {note1, startPoseY};
  Pose2d[] path3 = {startPoseY, startPose};

  /** Creates a new Test. */
  public Test(DriveBase driveBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FollowBezier(driveBase, path1, 200, true),
      new FollowBezier(driveBase, path2, 200, false)
    );
  }
}
