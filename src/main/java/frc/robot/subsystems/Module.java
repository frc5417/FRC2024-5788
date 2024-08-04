package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class Module {
  /** Creates a new Module. */

  public CANSparkMax angleMotor;
  public CANSparkMax driveMotor;

  public final RelativeEncoder integratedDriveEncoder;
  private final RelativeEncoder integratedAngleEncoder;

  private final int moduleNum; // ZERO INDEXED

  private static final double kP = 0.50; //0.4
  private static final double kI = 0.0;
  private static final double kD = 0.013; //0.006

  public final PIDController pid = new PIDController(kP, kI, kD);

  private Boolean invertDriveSpeed = false;
  
  private CANcoder _CANCoder;
  private CANcoderConfigurator configurator;

  int cnt = 0;

  public Module(int module, boolean inverted) {
    
    this.moduleNum = module;

     /* Angle Motor Config */
     angleMotor = new CANSparkMax(Constants.ModuleConstants.angleMotorIDS[this.moduleNum], MotorType.kBrushless);
    //  angleMotor.setIdleMode(IdleMode.kBrake);
     configAngleMotor();

     integratedAngleEncoder = angleMotor.getEncoder();
     angleMotor.getPIDController();

     _CANCoder = new CANcoder(Constants.ModuleConstants.CANCoderID[this.moduleNum], "canivore");
     configurator = _CANCoder.getConfigurator();

    //  _CANCoder.setPositionToAbsolute(0);
     configurator.apply(returnCANConfig());
    //  _CANCoder.setPosition(0);
    
     /* Drive Motor Config */
     driveMotor = new CANSparkMax(Constants.ModuleConstants.driveMotorIDS[this.moduleNum], MotorType.kBrushless);
     configDriveMotor();

     integratedDriveEncoder = driveMotor.getEncoder();
     driveMotor.getPIDController();
     integratedDriveEncoder.setPosition(0);
     

    pid.enableContinuousInput(0, Math.PI * 2);
    pid.setTolerance(0.0);

    this.invertDriveSpeed = inverted;
    // if(_CANCoder.getMagnetFieldStrength() != MagnetFieldStrength.Good_GreenLED) {
      // throw new RuntimeException("CANCoder on Module #" + Integer.valueOf(this.moduleNum).toString() + " is not green!");
    // }
  }

  public void setSpeedAndAngle(ModuleState targetState) {
    double x = setAngle(targetState.getDir());
    double y = setDriveSpeed(targetState.getVel());

    // if (++cnt % 50 == 0) {
    //   System.out.printf("Set module %d angle to %f, speed to %f\n", this.moduleNum, x, y);
    // }
  }

  //angle to normalize between 0 and 2PI RAD
  public double normalizeAngle(double angle) {
    double fixedAngle = angle;
    while (fixedAngle > (2*Math.PI)) { 
      fixedAngle -= (2*Math.PI); 
    }
    while (fixedAngle < 0.0 ) { 
      fixedAngle += (2*Math.PI); 
    }
    return fixedAngle;
  }

  public double setDriveSpeed(double speed) {
    int invertMultiplier = 1;
    if (invertDriveSpeed) { 
      invertMultiplier = -1; 
    }

    double x = speed * invertMultiplier;
    
    driveMotor.set(x);
    return x;
  }

  public double getAngleInRadians() { 
    return (_CANCoder.getAbsolutePosition().getValueAsDouble() * 360.0 - Constants.ModuleConstants.motorDegrees[this.moduleNum]) * (Math.PI/180.0);
  }

  public double getAngle() {
    return _CANCoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public double getDriveVelocity() {
    return integratedDriveEncoder.getVelocity();
  }

  public double getAngularVelocity() {
    return integratedAngleEncoder.getVelocity();
  }

  private void invertDrive() {
    invertDriveSpeed = invertDriveSpeed == false;
  }

  public double setAngle(double angle_in_rad) {
    
    // Rotation2d desiredState = Rotation2d.fromRadians(MathUtil.inputModulus(angle_in_rad, -Math.PI/2, Math.PI/2));
    // var delta = desiredState.minus(Rotation2d.fromRadians(MathUtil.inputModulus(this.getAngleInRadians(), -Math.PI/2, Math.PI/2)));
    // if (Math.abs(delta.getDegrees()) > 90.0) {
    //   invertDrive();
    //   desiredState = desiredState.rotateBy(Rotation2d.fromDegrees(180));
    // } 
    // angle_in_rad = desiredState.getRadians();
    // if (angle_in_rad < 0) {
    //   angle_in_rad += 2 * Math.PI;
    // }


    double x = (angle_in_rad);

    pid.setSetpoint(angle_in_rad);

    // String name = "Mod" + String.valueOf(this.moduleNum);
    // SmartDashboard.putNumber(name, this.getAngleInRadians());

    if (Math.abs(this.pid.getSetpoint() - this.getAngleInRadians()) > (Constants.ModuleConstants.degTolerance*(Math.PI/180))) {
      this.angleMotor.set(MathUtil.clamp(this.pid.calculate(this.getAngleInRadians()), -1, 1));
    } else {
      this.angleMotor.set(0.0);
    }

    return x;
  }

  public void resetDriveAngleEncoder() {
    _CANCoder.close();
  }

  public CANcoderConfiguration returnCANConfig() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    return canConfig;
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    // angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    // angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    angleMotor.setSmartCurrentLimit(25);
    // integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    // angleController.setP(Constants.Swerve.angleKP);
    // angleController.setI(Constants.Swerve.angleKI);
    // angleController.setD(Constants.Swerve.angleKD);
    // angleController.setFF(Constants.Swerve.angleKFF);
    // angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    driveMotor.setSmartCurrentLimit(50);
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    // driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    // driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    // driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    // driveController.setP(Constants.Swerve.angleKP);
    // driveController.setI(Constants.Swerve.angleKI);
    // driveController.setD(Constants.Swerve.angleKD);
    // driveController.setFF(Constants.Swerve.angleKFF);
    // driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
  }

  public static class ModuleState {
    private final double m_vel;
    private final double m_dir;

    public ModuleState(double vel, double dir) {
        m_vel = vel;
        m_dir = dir;
    }

    public double getVel() {
        return m_vel;
    }

    public double getDir() {
        return m_dir;
    }
  }
}
