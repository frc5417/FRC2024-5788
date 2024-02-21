package frc.robot.subsystems;
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
import com.kauailabs.navx.frc.AHRS;

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

    private Field2d field = new Field2d();





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
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1])),
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3]))
            });


    
    } 

    // public Pose2d getCurrentPose() {
    //     return globalPose;
    // }

    public Pose2d getCurrentPose() {
        Pose2d pose = m_sdkOdom.getPoseMeters().times(-1);
        return m_sdkOdom.getPoseMeters();
        // return new Pose2d(globalPose.getX(), globalPose.getY(), new Rotation2d(globalPose.getRotation().getRadians()+Math.PI/2));
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        // return new ChassisSpeeds(m_ahrs.getVelocityY(), -1 * m_ahrs.getVelocityX(), m_ahrs.getRate() * (Math.PI/180.0));
        SwerveModuleState[] states = new SwerveModuleState[moduleGroup.length];
        states[0] = new SwerveModuleState(moduleGroup[0].getDriveVelocity(), Rotation2d.fromRadians(moduleGroup[0].getAngleInRadians())); //The velocity is RPM so convert to M/S
        states[1] = new SwerveModuleState(moduleGroup[1].getDriveVelocity(), Rotation2d.fromRadians(moduleGroup[1].getAngleInRadians()));//The velocity is RPM so convert to M/S
        states[2] = new SwerveModuleState(moduleGroup[2].getDriveVelocity(), Rotation2d.fromRadians(moduleGroup[2].getAngleInRadians()));//The velocity is RPM so convert to M/S
        states[3] = new SwerveModuleState(moduleGroup[3].getDriveVelocity(), Rotation2d.fromRadians(moduleGroup[3].getAngleInRadians()));//The velocity is RPM so convert to M/S

        return new ChassisSpeeds(); //m_skdKine.toChassisSpeeds(states);
    }

    public boolean shouldFlipPath() {
        // Optional<Alliance> ally = DriverStation.getAlliance();
        // if (ally.isPresent()) {
        //     if (ally.get() == Alliance.Red) {
        //         return Constants.Swerve.shouldFlipAuto;
        //     }
        //     if (ally.get() == Alliance.Blue) {
        //         return !Constants.Swerve.shouldFlipAuto;
        //     }
        // } else {
        //     System.out.println("ERROR: You are a bozo");
        //     return false;
        // }
        // System.out.println("ERROR: You are a bozo");
        return false;
    }

    public void resetOdometry(Pose2d pose) {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getAngleInRadians());//smallestAngle(moduleGroup[i].getAngleInRadians()*(180.0/Math.PI)) * (Math.PI/180.0);
        }
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1])),
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3]))
        };
        m_sdkOdom.resetPosition(m_ahrs.getRotation2d(), modulePositions, pose);
    }

    public void setHardStates(Module.ModuleState[] targetState) {
        targetModuleStates = targetState;
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void setAutoSpeed(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        // ChassisSpeeds targetSpeeds = new ChassisSpeeds();
        autoSetSpeed = targetSpeeds;
        // ChassisSpeeds computed = RobotContainer.getSaturatedSpeeds(MathUtil.clamp(autoSetSpeed.vxMetersPerSecond, -1, 1), MathUtil.clamp(autoSetSpeed.vyMetersPerSecond, -1, 1), MathUtil.clamp(autoSetSpeed.omegaRadiansPerSecond, -1, 1)); //
        targetModuleStates = m_kinematics.getComputedModuleStates(targetSpeeds);
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

    @Override
    public void periodic() {
        // RobotContainer.m_photonsubsystem.updatePose();
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getAngleInRadians());//smallestAngle(moduleGroup[i].getAngleInRadians()*(180.0/Math.PI)) * (Math.PI/180.0);
        }
        

        m_sdkOdom.update(m_ahrs.getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(Math.abs(odomDeltas[0]), new Rotation2d(odomAngles[0])),
            new SwerveModulePosition(Math.abs(odomDeltas[1]), new Rotation2d(odomAngles[1])),
            new SwerveModulePosition(Math.abs(odomDeltas[2]), new Rotation2d(odomAngles[2])),
            new SwerveModulePosition(Math.abs(odomDeltas[3]), new Rotation2d(odomAngles[3]))
        });
        


        // field.setRobotPose(m_sdkOdom.getPoseMeters());


        SmartDashboard.putNumber("Yaw", m_ahrs.getYaw());
        SmartDashboard.putNumber("Pitch", m_ahrs.getPitch());
        SmartDashboard.putNumber("Roll", m_ahrs.getRoll());
        SmartDashboard.putNumber("Rotations", m_ahrs.getAngle());



        SmartDashboard.putNumber("Mod1_theta", -Math.abs(Math.toDegrees(odomAngles[0]))-90);
        SmartDashboard.putNumber("Mod2_theta", -Math.abs(Math.toDegrees(odomAngles[1]))-90);
        SmartDashboard.putNumber("Mod3_theta", -Math.abs(Math.toDegrees(odomAngles[2]))-90);
        SmartDashboard.putNumber("Mod4_theta", -Math.abs(Math.toDegrees(odomAngles[3]))-90);

        Pose2d pose = m_sdkOdom.getPoseMeters().times(-1);
        
        SmartDashboard.putNumber("GLOBAL POSE X: ", pose.getX());
        SmartDashboard.putNumber("GLOBAL POSE Y: ", pose.getY());

        SmartDashboard.updateValues();
        
    }
}
