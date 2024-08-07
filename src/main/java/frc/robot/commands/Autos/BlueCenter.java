// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import frc.robot.CustomNamedCommands;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueCenter extends SequentialCommandGroup {
  /** Creates a new Test. */
  public BlueCenter(DriveBase driveBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d startPoseY = new Pose2d(1.2, 0, Rotation2d.fromDegrees(0));
    Pose2d startPose2 = new Pose2d(-0.1, 0, Rotation2d.fromDegrees(0));


    Pose2d[] path1 = {startPose, startPoseY};
    Pose2d[] path2 = {startPoseY, startPose2};

    // FollowBezier follow1 = new FollowBezier(driveBase, path1, 200, true);

    addCommands(
      new ResetOdom(driveBase, new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
      new Fire(1.5),
      CustomNamedCommands.getCommand("IntakeOn"),
      CustomNamedCommands.getCommand("IndexOn"),
      new FollowBezier(driveBase, path1, 120, false),
      CustomNamedCommands.getCommand("IndexOff"),
      CustomNamedCommands.getCommand("IntakeOff"),
      new FollowBezier(driveBase, path2, 120, false),
      CustomNamedCommands.getCommand("IndexReverse"),
      new WaitCommand(0.5),
      new Fire(3)
    );
  }
}
