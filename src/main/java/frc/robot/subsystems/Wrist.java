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

  public double wristGoHere = 0;

  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);

  public final PIDController wristPID = new PIDController(p, i, d);

  public Wrist() {
    wristMotor = new CANSparkMax(Constants.ManipulatorConstants.wrist, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPosition(0);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristPID.setTolerance(0.005);
  }

  public void wristSpinny(double spinnyPower) {
    wristMotor.set(spinnyPower * 0.75);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
