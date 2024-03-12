package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Autos.FollowBezier;
import frc.robot.subsystems.DriveBase;

import frc.robot.subsystems.Intake;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Intake m_intake;
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(new PIDConstants(2.0, 0.5, 0.0), new PIDConstants(0.05, 0.0, 0.0), Constants.Swerve.maxModuleSpeed, Constants.DriveBaseConstants.driveBaseRadius, new ReplanningConfig());
    // private FollowBezier follower;

    public AutonLoader(DriveBase driveBase, Intake intake) {

        m_driveBase = driveBase;
        m_intake = intake;
        // follower = new FollowBezier(m_driveBase);


        AutoBuilder.configureHolonomic(m_driveBase::getCurrentPose, m_driveBase::resetOdometry, m_driveBase::getRobotRelativeChassisSpeeds, m_driveBase::setAutoSpeed, holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);

    }

    public Command getAuton() {
        // Pose2d[] path = {new Pose2d(0, 0, new Rotation2d()), new Pose2d(0, 10, new Rotation2d()), new Pose2d(5, 5, new Rotation2d()), new Pose2d(10, 15, new Rotation2d())};
        // follower.setPath(path, 20);
        // return follower;
        return new PathPlannerAuto("2Note_He");
    }    
}