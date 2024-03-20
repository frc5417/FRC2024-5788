// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.ArrayList;
import java.util.List;

import javax.print.attribute.standard.MediaSize.NA;
import javax.swing.GroupLayout.ParallelGroup;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

public class TwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new TwoNoteCenter. */
  public TwoNoteCenter(TargetStateRun targetStateRun, PhotonSubsystem photon, Pose2d startingPose) {
    // A_Star pathSolver = new A_Star(Constants.Auton.robot_size, Constants.Auton.field_size);
    // Pose2d startPose = RobotContainer.WPI_to_Custom(photon.getEstimatedFieldPose());
    Pose2d startPose = new Pose2d(0, 0, new Rotation2d());
    targetStateRun.m_drivebase.resetOdometry(startPose);

    Pose2d[] path_back = new Pose2d[] {startPose, new Pose2d(startPose.getX(), startPose.getY()+2.0, new Rotation2d())};
    Pose2d[] path_for = new Pose2d[] {new Pose2d(0, 2, new Rotation2d()), new Pose2d(0, 0, new Rotation2d())};

    FollowBezier back = new FollowBezier(targetStateRun);
    FollowBezier forw = new FollowBezier(targetStateRun);
    back.setPath(path_back, 100);
    forw.setPath(path_for, 100);

    RobotContainer.defineNamedCommands();

    addCommands(
      CustomNamedCommands.getCommand("ShootWrist"),
      CustomNamedCommands.getCommand("ShooterOn"),
      new WaitCommand(1),
      CustomNamedCommands.getCommand("IndexOn"),
      new WaitCommand(1.5),
      CustomNamedCommands.getCommand("IndexOff"),
      CustomNamedCommands.getCommand("ShooterOff"),
      CustomNamedCommands.getCommand("HandoffWrist"),
      new ParallelCommandGroup(
        back,
        CustomNamedCommands.getCommand("IntakeOn"),
        CustomNamedCommands.getCommand("IndexOn")
      ),
      CustomNamedCommands.getCommand("IndexReverse"),
      new WaitCommand(0.3),
      CustomNamedCommands.getCommand("IndexOff"),
      forw,
      CustomNamedCommands.getCommand("ShootWrist"),
      CustomNamedCommands.getCommand("ShooterOn"),
      new WaitCommand(1.5),
      CustomNamedCommands.getCommand("IndexOn"),
      new WaitCommand(5),
      CustomNamedCommands.getCommand("ShooterOff"),
      CustomNamedCommands.getCommand("IndexOff"),
      CustomNamedCommands.getCommand("IntakeOff")
    );
  }
}
