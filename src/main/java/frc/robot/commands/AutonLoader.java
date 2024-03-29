package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Autos.BlueFarSide;
import frc.robot.commands.Autos.BlueNotStupid;
import frc.robot.commands.Autos.Fire;
import frc.robot.commands.Autos.RedFarSide;
import frc.robot.commands.Autos.RedNotStupid;
import frc.robot.subsystems.DriveBase;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.TargetStateRun;
import frc.robot.subsystems.A_Star.A_Star;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Intake m_intake;
    private final TargetStateRun m_targetStateRun;
    private final PhotonSubsystem m_photonsubsystem;
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(new PIDConstants(2.0, 0.5, 0.0), new PIDConstants(0.05, 0.0, 0.0), Constants.Swerve.maxModuleSpeed, Constants.DriveBaseConstants.driveBaseRadius, new ReplanningConfig());
    // private BlueFarSide blueFarSide;
    // private RedFarSide redFarSide;
    // private RedNotStupid redNotStupid;
    private BlueNotStupid blueNotStupid;

    

    public AutonLoader(DriveBase driveBase, Intake intake, TargetStateRun targetStateRun, PhotonSubsystem photon) {
        m_targetStateRun = targetStateRun;
        m_driveBase = driveBase;
        m_intake = intake;
        m_photonsubsystem = photon;

        A_Star.rectangularObstacle(Constants.Auton.BlueObstacle_TopLeft, Constants.Auton.BlueObstacle_BottomRight);
        A_Star.rectangularObstacle(Constants.Auton.RedObstacle_TopLeft, Constants.Auton.RedObstacle_BottomRight);

        // blueFarSide = new BlueFarSide(targetStateRun, m_photonsubsystem);
        // redFarSide = new RedFarSide(targetStateRun, photon);
        // redNotStupid = new RedNotStupid(targetStateRun, photon);
        blueNotStupid = new BlueNotStupid(targetStateRun, photon);
        

        
        

        AutoBuilder.configureHolonomic(m_driveBase::getCurrentPose, m_driveBase::resetOdometry, m_driveBase::getRobotRelativeChassisSpeeds, m_driveBase::setAutoSpeed, holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);

    }

    public Command getAuton() {
        // Pose2d.setPath(path, 200);
        return blueNotStupid;
        // return new PathPlannerAuto("2Note_He");
    }    
}