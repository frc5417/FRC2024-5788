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
public class BlueCenter3Note extends SequentialCommandGroup {
  /** Creates a new Test. */
  public BlueCenter3Note(DriveBase driveBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Pose2d startPose = new Pose2d(1.25, 5.5, Rotation2d.fromDegrees(0));
    Pose2d startPoseY = new Pose2d(3, 5.5, Rotation2d.fromDegrees(0));
    Pose2d startPose2 = new Pose2d(1.2, 5.5, Rotation2d.fromDegrees(0));


    Pose2d[] path1 = {startPose, startPoseY};
    Pose2d[] path2 = {startPoseY, startPose2};

    Pose2d otherNote = new Pose2d(2.8, 4.1, Rotation2d.fromDegrees(0));

    Pose2d[] path3 = {startPose2, otherNote};
    Pose2d[] path4 = {otherNote, startPose2};

    // driveBase.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

    // FollowBezier follow1 = new FollowBezier(driveBase, path1, 200, true);

    addCommands(
      new ResetOdom(driveBase, startPose),
      new Fire(0.2),
      CustomNamedCommands.getCommand("IntakeOn"),
      CustomNamedCommands.getCommand("IndexOn"),
      new FollowBezier(driveBase, path1, 10, false),
      CustomNamedCommands.getCommand("IndexOff"),
      CustomNamedCommands.getCommand("IntakeOff"),
      new FollowBezier(driveBase, path2, 10, false),
      CustomNamedCommands.getCommand("IndexReverse"),
      new WaitCommand(0.5),
      new Fire(3),
      //---------------------------------
      CustomNamedCommands.getCommand("IntakeOn"),
      CustomNamedCommands.getCommand("IndexOn"),
      new FollowBezier(driveBase, path3, 70, false),
      CustomNamedCommands.getCommand("IndexOff"),
      CustomNamedCommands.getCommand("IntakeOff"),
      new FollowBezier(driveBase, path4, 70, false),
      CustomNamedCommands.getCommand("IndexReverse"),
      new WaitCommand(0.5),
      new Fire(3)
    );
  }
}
