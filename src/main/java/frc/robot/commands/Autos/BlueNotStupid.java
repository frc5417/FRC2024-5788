// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.Collections;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.CustomNamedCommands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.TargetStateRun;
import frc.robot.subsystems.A_Star.A_Star;
import frc.robot.subsystems.A_Star.Node;

public class BlueNotStupid extends SequentialCommandGroup {
  /** Creates a new TwoNoteCenter. */
  public BlueNotStupid(TargetStateRun targetStateRun, PhotonSubsystem photon) {
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
    targetStateRun.m_drivebase.resetOdometry(startPose);
    

    Pose2d[] path_back = {startPose, new Pose2d(2.0, 0, Rotation2d.fromDegrees(45)), new Pose2d(2.0, 8, Rotation2d.fromDegrees(45))}; //x=11 is back
    Pose2d[] path_for = {new Pose2d(2.0, 8, Rotation2d.fromDegrees(45)), new Pose2d(2.0, 0, Rotation2d.fromDegrees(45)), startPose};



    // Pose2d[] path_for = A_Star.nodeListToPosesWPI(A_Star.compute(new Node(notePoseCustom.getX(), notePoseCustom.getY(), false), new Node(startPoseCustom.getX(), startPoseCustom.getY(), false)));


    // for (int i=0; i<path_back.length; i++) {
    //   path_for[path_for.length-1-i] = path_back[i];
    // }


    // A_Star.setRotation(path_for, path_for.length-1, Rotation2d.fromDegrees(45-90));
    // path_back[0] = new Pose2d(path_back[0].getTranslation(), Rotation2d.fromDegrees(45-90));
    // path_for[path_for.length-1] = new Pose2d(path_for[path_for.length-1].getTranslation(), Rotation2d.fromDegrees(45-90));
    FollowBezier back = new FollowBezier(targetStateRun);
    FollowBezier forw = new FollowBezier(targetStateRun);
    // FollowBezier forw = new FollowBezier(targetStateRun);
    back.setPath(path_back, 100);
    forw.setPath(path_for, 100);

    RobotContainer.defineNamedCommands();

    // targetStateRun.m_drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()));

    // FollowBezier path = new FollowBezier(targetStateRun);
    // Pose2d[] work = new Pose2d[] {new Pose2d(0, 0, new Rotation2d()), new Pose2d(4, 4, new Rotation2d())};
    // path.setPath(work, 100);
    // addCommands(path);
    // addCommands(back, forw);
    addCommands(new Fire(1.5).andThen(back.alongWith(CustomNamedCommands.getCommand("IntakeOn")).andThen(CustomNamedCommands.getCommand("IntakeOff")).andThen(forw)));
    /*
    addCommands(
      // CustomNamedCommands.getCommand("ShootWrist"),
      // CustomNamedCommands.getCommand("ShooterOn"),
      // new WaitCommand(1),
      // CustomNamedCommands.getCommand("IndexOn"),
      // new WaitCommand(1.5),
      // CustomNamedCommands.getCommand("IndexOff"),
      // CustomNamedCommands.getCommand("ShooterOff"),
      new Fire(1.5).andThen(
      CustomNamedCommands.getCommand("HandoffWrist").andThen(
      new ParallelCommandGroup(
        back.withTimeout(4),
        CustomNamedCommands.getCommand("IntakeOn"),
        CustomNamedCommands.getCommand("IndexOn")
      ).andThen(
      CustomNamedCommands.getCommand("IndexReverse").andThen(
      new WaitCommand(0.3).andThen(
      CustomNamedCommands.getCommand("IndexOff").andThen(
      forw.withTimeout(4).andThen(
      // CustomNamedCommands.getCommand("ShootWrist"),
      // CustomNamedCommands.getCommand("ShooterOn"),
      // new WaitCommand(1.5),
      // CustomNamedCommands.getCommand("IndexOn"),
      // new WaitCommand(5),
      // CustomNamedCommands.getCommand("ShooterOff"),
      // CustomNamedCommands.getCommand("IndexOff"),
      new Fire(5).andThen(
      CustomNamedCommands.getCommand("IntakeOff")))))))))
    ); */
  } 
}
