// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.A_Star.A_Star;
import frc.robot.subsystems.A_Star.Node;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedSourceSide extends SequentialCommandGroup {
  /** Creates a new RedSourceSide. */
  public RedSourceSide(DriveBase driveBase) {
    // Add your commands in the addCommands() call, e.g.
    Pose2d startpose = RobotContainer.WPI_to_Custom(new Pose2d(15.8, 4.5, Rotation2d.fromDegrees(-135)));
    Pose2d midfield = RobotContainer.WPI_to_Custom(new Pose2d(8, 0.75, Rotation2d.fromDegrees(180)));
    Pose2d endpose = RobotContainer.WPI_to_Custom(new Pose2d(15.8, 4.5, Rotation2d.fromDegrees(-135)));

    Pose2d[] toMidFieldPath = A_Star.nodeListToPosesWPI(A_Star.compute(new Node(startpose.getTranslation(), false), new Node(midfield.getTranslation(), false)), Rotation2d.fromDegrees(-135));
    Pose2d[] toStartFromMidPath = A_Star.nodeListToPosesWPI(A_Star.compute(new Node(midfield.getTranslation(), false), new Node(endpose.getTranslation(), false)), Rotation2d.fromDegrees(180));

    toMidFieldPath = A_Star.setRotation(toMidFieldPath, -1, Rotation2d.fromDegrees(180));
    toStartFromMidPath = A_Star.setRotation(toStartFromMidPath, 0, Rotation2d.fromDegrees(-135));

    FollowBezier toMidField = new FollowBezier(driveBase, toMidFieldPath, 500, true);
    FollowBezier toStartFromMid = new FollowBezier(driveBase, toStartFromMidPath, 500, false);
    addCommands(
      new Fire(1.5),
      toMidField,
      toStartFromMid
      );
  }
}
