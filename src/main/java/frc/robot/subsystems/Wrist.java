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

  // Prelim vars for PID

  // private double wristGoHere = 0;

  // private static final double kP = 0;
  // private static final double kI = 0;
  // private static final double kD = 0;

  // public final PIDController wristPID = new PIDController(kP, kI, kD);

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
