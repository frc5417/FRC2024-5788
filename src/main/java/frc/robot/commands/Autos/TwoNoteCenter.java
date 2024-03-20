// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CustomNamedCommands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoControllers.FollowBezier;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.TargetStateRun;

public class TwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new TwoNoteCenter. */
  public TwoNoteCenter(TargetStateRun targetStateRun, PhotonSubsystem photon, Pose2d startingPose) {
    // A_Star pathSolver = new A_Star(Constants.Auton.robot_size, Constants.Auton.field_size);
    // Pose2d startPose = RobotContainer.WPI_to_Custom(photon.getEstimatedFieldPose());
    Pose2d startPose = new Pose2d(15.0, 4.5, Rotation2d.fromDegrees(-90));
    targetStateRun.m_drivebase.resetOdometry(RobotContainer.WPI_to_Custom(startPose));

    

    Pose2d[] path_back = new Pose2d[] {startPose, new Pose2d(3.5, 5.5, new Rotation2d())};
    Pose2d[] path_for = new Pose2d[] {path_back[1], startPose};

    FollowBezier back = new FollowBezier(targetStateRun);
    FollowBezier forw = new FollowBezier(targetStateRun);
    back.setPath(path_back, 100);
    forw.setPath(path_for, 100);

    RobotContainer.defineNamedCommands();

    addCommands(
      // CustomNamedCommands.getCommand("ShootWrist"),
      // CustomNamedCommands.getCommand("ShooterOn"),
      // new WaitCommand(1),
      // CustomNamedCommands.getCommand("IndexOn"),
      // new WaitCommand(1.5),
      // CustomNamedCommands.getCommand("IndexOff"),
      // CustomNamedCommands.getCommand("ShooterOff"),
      new Fire(1.5),
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
      // CustomNamedCommands.getCommand("ShootWrist"),
      // CustomNamedCommands.getCommand("ShooterOn"),
      // new WaitCommand(1.5),
      // CustomNamedCommands.getCommand("IndexOn"),
      // new WaitCommand(5),
      // CustomNamedCommands.getCommand("ShooterOff"),
      // CustomNamedCommands.getCommand("IndexOff"),
      new Fire(5),
      CustomNamedCommands.getCommand("IntakeOff")
    );
  }
}
