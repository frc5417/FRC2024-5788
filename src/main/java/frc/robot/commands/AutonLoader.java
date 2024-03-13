package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Autos.TwoNoteCenter;
import frc.robot.subsystems.DriveBase;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TargetStateRun;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Intake m_intake;
    private final TargetStateRun m_targetStateRun;
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(new PIDConstants(2.0, 0.5, 0.0), new PIDConstants(0.05, 0.0, 0.0), Constants.Swerve.maxModuleSpeed, Constants.DriveBaseConstants.driveBaseRadius, new ReplanningConfig());
    private TwoNoteCenter follower;

    

    public AutonLoader(DriveBase driveBase, Intake intake, TargetStateRun targetStateRun) {
        m_targetStateRun = targetStateRun;
        m_driveBase = driveBase;
        m_intake = intake;
        follower = new TwoNoteCenter(targetStateRun);


        AutoBuilder.configureHolonomic(m_driveBase::getCurrentPose, m_driveBase::resetOdometry, m_driveBase::getRobotRelativeChassisSpeeds, m_driveBase::setAutoSpeed, holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);

    }

    public Command getAuton() {
        // Pose2d.setPath(path, 200);
        return follower;
        // return new PathPlannerAuto("2Note_He");
    }    
}