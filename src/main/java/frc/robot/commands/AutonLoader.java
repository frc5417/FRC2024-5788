package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private static SendableChooser<Command> chooser;
    private final ReplanningConfig replanningConfig = new ReplanningConfig();
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(Constants.Swerve.maxModuleSpeed, Constants.DriveBaseConstants.driveBaseRadius, replanningConfig);

    //PathPlanner auton groups
    private static PathPlannerPath trajectory = PathPlannerPath.fromPathFile("moose");

    // private static List<PathPlannerPath> sf8 = PathPlannerAuto.getPathGroupFromAutoFile("sf8");
    // private static List<PathPlannerPath> bozo = PathPlannerAuto.getPathGroupFromAutoFile("newsf");
    // private static List<PathPlannerPath> straightline = PathPlannerAuto.getPathGroupFromAutoFile("straightline");

    public AutonLoader(DriveBase driveBase) {

        m_driveBase = driveBase;
        chooser = new SendableChooser<>();       

        AutoBuilder.configureHolonomic(m_driveBase::getCurrentPose, m_driveBase::resetOdometry, m_driveBase::getRelativeChassisSpeeds, m_driveBase::setAutoSpeed, holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);

        
        
        // for (String path : Constants.Auton.paths) {
            // chooser.addOption(path, getAutonFromPath(path));
        // }

        // chooser.addOption("Single Score Mobility", m_autoncommands.MOBILITY);
        // chooser.addOption("Double Score", m_autoncommands.SCORING);

        // chooser.addOption("trajectory", AutoBuilder.followPath((PathPlannerPath) trajectory));
        // chooser.addOption("sf8", AutoBuilder.followPath((PathPlannerPath) sf8));
        // chooser.addOption("straightline", AutoBuilder.followPath((PathPlannerPath) straightline));
        // chooser.addOption("newsf", AutoBuilder.followPathWithEvents((PathPlannerPath) newsf));

        SmartDashboard.putData(chooser);
    }

    // private Command getAutonFromPath(String path) {
    //     return new PrintCommand(path);
    // }

    public Command getAuton() {
        // return chooser.getSelected();
        // return autoBuilder.fullAuto(pathGroup);
        // return AutoBuilder.buildAuto("test");
        m_driveBase.resetOdometry(new Pose2d(trajectory.getPoint(0).position, new Rotation2d(-90)));
        // return AutoBuilder.followPath(trajectory);
        return AutoBuilder.buildAuto("test");
    }    
}