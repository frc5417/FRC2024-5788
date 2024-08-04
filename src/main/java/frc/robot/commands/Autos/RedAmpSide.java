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
public class RedAmpSide extends SequentialCommandGroup {
  /** Creates a new Test. */
  public RedAmpSide(DriveBase driveBase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Pose2d startPose = new Pose2d(15.75, 6.5, Rotation2d.fromDegrees(120));
    Pose2d startPose1 = new Pose2d(15.75, 7, Rotation2d.fromDegrees(180));
    Pose2d startPose2 = new Pose2d(13.75, 7, Rotation2d.fromDegrees(180));


    Pose2d[] path1 = {startPose, startPose1, startPose2};
    Pose2d[] path2 = {startPose2, startPose};

    // FollowBezier follow1 = new FollowBezier(driveBase, path1, 200, true);

    addCommands(
      new ResetOdom(driveBase, startPose),
      new Fire(0.2),
      CustomNamedCommands.getCommand("IntakeOn"),
      CustomNamedCommands.getCommand("IndexOn"),
      new FollowBezier(driveBase, path1, 70, false),
      CustomNamedCommands.getCommand("IndexOff"),
      CustomNamedCommands.getCommand("IntakeOff"),
      new FollowBezier(driveBase, path2, 70, false),
      CustomNamedCommands.getCommand("IndexReverse"),
      new WaitCommand(0.5),
      new Fire(3)
    );
  }
}
