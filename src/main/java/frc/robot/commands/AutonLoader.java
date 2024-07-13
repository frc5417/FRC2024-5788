package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Autos.Fire;
import frc.robot.commands.Autos.HoloTune;
import frc.robot.commands.Autos.RedSourceSide;
import frc.robot.commands.Autos.Test;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.A_Star.A_Star;

public class AutonLoader {
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    

    public AutonLoader(DriveBase driveBase, Intake intake, PhotonSubsystem photon) {
        photon.setDriverMode(false);
        A_Star.rectangularObstacle(Constants.Auton.BlueObstacle_TopLeft, Constants.Auton.BlueObstacle_BottomRight);
        A_Star.rectangularObstacle(Constants.Auton.RedObstacle_TopLeft, Constants.Auton.RedObstacle_BottomRight);
        
        RobotContainer.defineNamedCommands();
        

        m_chooser.addOption("Fire 1.5", new Fire(1.5));
        m_chooser.addOption("Red Source Side", new RedSourceSide(driveBase));
        m_chooser.addOption("HoloTune", new HoloTune(driveBase));
        m_chooser.addOption("Test", new Test(driveBase));

        // SmartDashboard.putData(m_chooser);
        // SmartDashboard.updateValues();
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAuton() {
        return m_chooser.getSelected();
    }    
}