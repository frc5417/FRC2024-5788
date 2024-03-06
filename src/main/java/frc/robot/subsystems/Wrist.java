// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  private final CANSparkMax wristMotor;

  public final RelativeEncoder wristEncoder;
 

  public final PIDController wristPID = new PIDController(6.75, 0.0, 1.0);

  private double wristPos = 0.0;

  public Wrist() {
    wristMotor = new CANSparkMax(Constants.ManipulatorConstants.wrist, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPosition(0);
    wristEncoder.setPositionConversionFactor(1/20.0);
    wristMotor.setIdleMode(IdleMode.kBrake);

    // wristPID.setTolerance(0.0);
  }

  // public void wristSpinny(double spinnyPower) {
  //   wristMotor.set(spinnyPower * 0.75);
  // }

  public void setWristPos(double rot) {
    wristPos = rot;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    wristPID.setSetpoint(wristPos);
    // wristMotor.set(MathUtil.clamp(wristPID.calculate(wristEncoder.getPosition()), -1, 1));
    if (Math.abs(wristPID.getSetpoint() - wristEncoder.getPosition()) > Constants.ManipulatorConstants.wristTolerance) {
      wristMotor.set(wristPID.calculate(wristEncoder.getPosition()));
      SmartDashboard.putNumber("PID", 1);
    } else {
      wristMotor.set(0.0);
    }
    SmartDashboard.putNumber("WristPosition", wristEncoder.getPosition());    
    SmartDashboard.putNumber("Wrist Power", wristMotor.get());
    SmartDashboard.putNumber("PID", 0);

    SmartDashboard.updateValues();
  }
}