// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.ArrayList;
import java.util.List;

import javax.print.attribute.standard.MediaSize.NA;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.TargetStateRun;
import frc.robot.subsystems.A_Star.A_Star;
import frc.robot.subsystems.A_Star.Node;

public class TwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new TwoNoteCenter. */
  public TwoNoteCenter(TargetStateRun targetStateRun, PhotonSubsystem photon, Pose2d startingPose) {
    // A_Star pathSolver = new A_Star(Constants.Auton.robot_size, Constants.Auton.field_size);
    // Pose2d startPose = RobotContainer.WPI_to_Custom(photon.getEstimatedFieldPose());
    Pose2d startPose = new Pose2d(0, 0, new Rotation2d());
    targetStateRun.m_drivebase.resetOdometry(startPose);

    Pose2d[] path_back = new Pose2d[] {startPose, new Pose2d(startPose.getX(), startPose.getY()+2.0, new Rotation2d())};
    Pose2d[] path_for = new Pose2d[] {path_back[1], startPose};

    FollowBezier back = new FollowBezier(targetStateRun);
    FollowBezier forw = new FollowBezier(targetStateRun);
    back.setPath(path_back, 100);
    forw.setPath(path_for, 100);

    addCommands(
      NamedCommands.getCommand("ShootWrist"),
      NamedCommands.getCommand("ShooterOn"),
      new WaitCommand(1),
      NamedCommands.getCommand("IndexOn"),
      new WaitCommand(1.5),
      NamedCommands.getCommand("IndexOff"),
      NamedCommands.getCommand("ShooterOff"),
      NamedCommands.getCommand("HandoffWrist"),
      deadlineWith(
        back,
        NamedCommands.getCommand("IntakeOn"),
        NamedCommands.getCommand("IndexOn")
      ),
      NamedCommands.getCommand("IndexReverse"),
      new WaitCommand(0.3),
      NamedCommands.getCommand("IndexOff"),
      forw,
      NamedCommands.getCommand("ShootWrist"),
      NamedCommands.getCommand("ShooterOn"),
      new WaitCommand(1.5),
      NamedCommands.getCommand("IndexOn"),
      new WaitCommand(5),
      NamedCommands.getCommand("ShooterOff"),
      NamedCommands.getCommand("IndexOff"),
      NamedCommands.getCommand("IntakeOff")
    );
  }
}
