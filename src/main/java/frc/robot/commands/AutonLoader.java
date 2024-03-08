package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Intake;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Intake m_intake;
    private static SendableChooser<Command> chooser;
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(new PIDConstants(2.0, 0.5, 0.0), new PIDConstants(0.1, 0.0, 0.0), Constants.Swerve.maxModuleSpeed, Constants.DriveBaseConstants.driveBaseRadius, new ReplanningConfig());

    //PathPlanner auton groups

    // private static List<PathPlannerPath> sf8 = PathPlannerAuto.getPathGroupFromAutoFile("sf8");
    // private static List<PathPlannerPath> bozo = PathPlannerAuto.getPathGroupFromAutoFile("newsf");
    // private static List<PathPlannerPath> straightline = PathPlannerAuto.getPathGroupFromAutoFile("straightline");

    private final SendableChooser<Command> autoChooser;

    public AutonLoader(DriveBase driveBase, Intake intake) {

        m_driveBase = driveBase;
        m_intake = intake;


        AutoBuilder.configureHolonomic(m_driveBase::getCurrentPose, m_driveBase::resetOdometry, m_driveBase::getRobotRelativeChassisSpeeds, m_driveBase::setAutoSpeed, holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);


        autoChooser = AutoBuilder.buildAutoChooser();
        
        // for (String path : Constants.Auton.paths) {
            // chooser.addOption(path, getAutonFromPath(path));
        // }

        // chooser.addOption("Single Score Mobility", m_autoncommands.MOBILITY);
        // chooser.addOption("Double Score", m_autoncommands.SCORING);

        // chooser.addOption("trajectory", AutoBuilder.followPath((PathPlannerPath) trajectory));
        // chooser.addOption("sf8", AutoBuilder.followPath((PathPlannerPath) sf8));
        // chooser.addOption("straightline", AutoBuilder.followPath((PathPlannerPath) straightline));
        // chooser.addOption("newsf", AutoBuilder.followPathWithEvents((PathPlannerPath) newsf));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // private Command getAutonFromPath(String path) {
    //     return new PrintCommand(path);
    // }

    public Command getAuton() {
        // return chooser.getSelected();
        // return autoBuilder.fullAuto(pathGroup);
        // return AutoBuilder.buildAuto("test");
        // m_driveBase.resetOdometry(trajectory.getPathPoses().get(0));
        // return AutoBuilder.followPath(trajectory);
        // return autoChooser.getSelected();
        // m_driveBase.resetOdometry(new Pose2d(5.0, 5.0, new Rotation2d()));
        return new PathPlannerAuto("test");
    }    
}