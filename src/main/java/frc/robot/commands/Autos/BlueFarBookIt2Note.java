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
public class BlueFarBookIt2Note extends SequentialCommandGroup {
  /** Creates a new Test. */
  public BlueFarBookIt2Note(DriveBase driveBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Pose2d startPose = new Pose2d(0.8, 4.60, Rotation2d.fromDegrees(-60));
    Pose2d pose2 = new Pose2d(2.6, 1.2, Rotation2d.fromDegrees(0));
    Pose2d pose3 = new Pose2d(7.8, 0.75, Rotation2d.fromDegrees(0));


    Pose2d[] path1 = {startPose, pose2, pose3};

    Pose2d pose4 = new Pose2d(8.3, 0.75, Rotation2d.fromDegrees(0));


    Pose2d[] path2 = {pose3, pose4};

    Pose2d[] path3 = {pose4, startPose};

    // FollowBezier follow1 = new FollowBezier(driveBase, path1, 200, true);

    addCommands(
      new ResetOdom(driveBase, new Pose2d(0.8, 4.60, Rotation2d.fromDegrees(-60))),
      new Fire(1.5),
      new FollowBezier(driveBase, path1, 120, false),
      CustomNamedCommands.getCommand("IntakeOn"),
      new FollowBezier(driveBase, path2, 1, false),
      CustomNamedCommands.getCommand("IntakeOff"),
      new FollowBezier(driveBase, path3, 70, false),
      new Fire(0.2)
    );
  }
}
