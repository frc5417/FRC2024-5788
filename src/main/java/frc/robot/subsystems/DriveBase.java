package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends SubsystemBase {

    private static Module.ModuleState targetModuleStates[];
    private final Kinematics m_kinematics;
    private final AHRS m_ahrs;

    public static Module[] moduleGroup;

    public static double[] odomDeltas = {0, 0, 0, 0};
    public static double[] odomPrevDeltas = {0, 0, 0, 0};
    public static double[] odomAngles = {0, 0, 0, 0};
    public static double[] encoderOffset = {0, 0, 0, 0};
    public static double[] encoderDriveOffset = {0, 0, 0, 0};

    Field2d field = new Field2d();


    double mod1Prev = 0;
    double mod1Curr = 0;
    int counter = 0;

    double tic, toc = 0;

    Translation2d m_frontLeftLocation = new Translation2d(-0.23495, 0.23495);
    Translation2d m_frontRightLocation = new Translation2d(0.23495, 0.23495);
    Translation2d m_backLeftLocation = new Translation2d(-0.23495, -0.23495);
    Translation2d m_backRightLocation = new Translation2d(0.23495, -0.23495);

    SwerveDriveKinematics m_skdKine = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    SwerveDriveOdometry m_sdkOdom;


    // Pose2d globalPose = new Pose2d(0.0, 0.0, new Rotation2d());
    // double X = 0.0;
    // double Y = 0.0;
    

    ChassisSpeeds autoSetSpeed = new ChassisSpeeds();

    private PIDController snapToNearestTheta = new PIDController(1, 0, 0);
    private boolean snappingOn = false;

    public DriveBase(Kinematics kinematics, AHRS ahrs) {
        m_kinematics = kinematics;
        m_ahrs = ahrs;

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++) {
            moduleGroup[i] = new Module(i, Constants.ModuleConstants.invertedMotors[i]);
            encoderOffset[i] = moduleGroup[i].getAngleInRadians();
            encoderDriveOffset[i] = moduleGroup[i].integratedDriveEncoder.getPosition();
        }

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.ModuleConstants.motorDegrees[i] * (Math.PI/180));

        m_sdkOdom = new SwerveDriveOdometry(
            m_skdKine, m_ahrs.getRotation2d(), new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0])),
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1]))
            });


    
    } 

    public Pose2d getCurrentPose() {
        Pose2d inverted = new Pose2d(m_sdkOdom.getPoseMeters().getY() * -1.0, m_sdkOdom.getPoseMeters().getX(), m_sdkOdom.getPoseMeters().getRotation());
        return inverted;
    }



    public boolean isRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void resetOdometry(Pose2d pose) {
        Pose2d inverted = new Pose2d(pose.getY(), pose.getX() * -1.0, pose.getRotation());
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getAngleInRadians());//smallestAngle(moduleGroup[i].getAngleInRadians()*(180.0/Math.PI)) * (Math.PI/180.0);
        }
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0])),
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1]))
        };
        m_sdkOdom.resetPosition(m_ahrs.getRotation2d(), modulePositions, inverted);
    }

    public void setHardStates(Module.ModuleState[] targetState) {
        targetModuleStates = targetState;
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    // public void setAutoSpeed(ChassisSpeeds chassisSpeeds) {
    //     ChassisSpeeds inverted = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond * -1.0, chassisSpeeds.vyMetersPerSecond * -1.0, chassisSpeeds.omegaRadiansPerSecond * -1.0); //TODO Make sure that this works with manson
    //     targetModuleStates = m_kinematics.getComputedModuleStates(inverted);
    // }
    public void setBlueAutoSpeed(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds inverted = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond * -1.0, chassisSpeeds.omegaRadiansPerSecond);
        targetModuleStates = m_kinematics.getComputedModuleStates(inverted);
    }

    public void setRedAutoSpeed(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds inverted = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond * -1.0, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        targetModuleStates = m_kinematics.getComputedModuleStates(inverted);
    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    public double smallestAngle(double largeAngle) {
        if(largeAngle > 0) {
            return largeAngle - Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI);
        } else {
            return (largeAngle + Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI)) + (2*Math.PI);
        }
    }

    public void setSnapping(boolean on_or_off) {
        snappingOn = on_or_off;
    }

    @Override
    public void periodic() {
        // RobotContainer.m_photonsubsystem.updatePose();
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getAngleInRadians());//smallestAngle(moduleGroup[i].getAngleInRadians()*(180.0/Math.PI)) * (Math.PI/180.0);
        }

        m_sdkOdom.update(m_ahrs.getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(Math.abs(odomDeltas[2]), new Rotation2d(odomAngles[2])),
            new SwerveModulePosition(Math.abs(odomDeltas[0]), new Rotation2d(odomAngles[0])),
            new SwerveModulePosition(Math.abs(odomDeltas[3]), new Rotation2d(odomAngles[3])),
            new SwerveModulePosition(Math.abs(odomDeltas[1]), new Rotation2d(odomAngles[1]))
        });

        if (snappingOn) {
            snapToNearestTheta.setSetpoint(m_ahrs.getYaw()-(m_ahrs.getYaw()%90.0));
            setDriveSpeed(RobotContainer.getSaturatedSpeeds(0, 0, snapToNearestTheta.calculate(m_ahrs.getYaw())));
        }
        
        field.setRobotPose(getCurrentPose());
        SmartDashboard.putData(field);

        // field.setRobotPose(m_sdkOdom.getPoseMeters());


        SmartDashboard.putNumber("Yaw", m_ahrs.getYaw());


        // SmartDashboard.putNumber("Mod1_theta", -Math.abs(Math.toDegrees(odomAngles[0]))-90);
        // SmartDashboard.putNumber("Mod2_theta", -Math.abs(Math.toDegrees(odomAngles[1]))-90);
        // SmartDashboard.putNumber("Mod3_theta", -Math.abs(Math.toDegrees(odomAngles[2]))-90);
        // SmartDashboard.putNumber("Mod4_theta", -Math.abs(Math.toDegrees(odomAngles[3]))-90);

        Pose2d pose = getCurrentPose();
        
        SmartDashboard.putNumber("GLOBAL POSE X: ", pose.getX());
        SmartDashboard.putNumber("GLOBAL POSE Y: ", pose.getY());
        SmartDashboard.putNumber("GLOBAL ROT", pose.getRotation().getDegrees());

        SmartDashboard.updateValues();
        
    }
}
