package frc.robot.commands;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Autos.Fire;
import frc.robot.commands.Autos.RedSourceSide;
import frc.robot.subsystems.DriveBase;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.A_Star.A_Star;

public class AutonLoader {
    private final DriveBase m_driveBase;
    private final Intake m_intake;
    private final PhotonSubsystem m_photonsubsystem;
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(new PIDConstants(2.0, 0.5, 0.0), new PIDConstants(0.05, 0.0, 0.0), Constants.Swerve.maxModuleSpeed, Constants.DriveBaseConstants.driveBaseRadius, new ReplanningConfig());
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    

    public AutonLoader(DriveBase driveBase, Intake intake, PhotonSubsystem photon) {
        m_driveBase = driveBase;
        m_intake = intake;
        m_photonsubsystem = photon;

        A_Star.rectangularObstacle(Constants.Auton.BlueObstacle_TopLeft, Constants.Auton.BlueObstacle_BottomRight);
        A_Star.rectangularObstacle(Constants.Auton.RedObstacle_TopLeft, Constants.Auton.RedObstacle_BottomRight);
        
        RobotContainer.defineNamedCommands();
        

        m_chooser.addOption("Fire 1.5", new Fire(1.5));
        m_chooser.addOption("Red Source Side", new RedSourceSide(driveBase));

        SmartDashboard.putData(m_chooser);
        SmartDashboard.updateValues();
    }

    public Command getAuton() {
        return m_chooser.getSelected();
    }    
}