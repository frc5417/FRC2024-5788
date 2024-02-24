// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final CANSparkMax elevatorMotor1;
  private final CANSparkMax elevatorMotor2;

  public final RelativeEncoder elevatorEncoder;


  public Elevator() {
    elevatorMotor1 = new CANSparkMax(Constants.ManipulatorConstants.elevatorMaster, MotorType.kBrushless);
    elevatorMotor2 = new CANSparkMax(Constants.ManipulatorConstants.elevatorSlave, MotorType.kBrushless);

    elevatorMotor2.setInverted(false);
    elevatorMotor1.setInverted(true);
    
    elevatorEncoder = elevatorMotor1.getEncoder();

    elevatorMotor1.setIdleMode(IdleMode.kBrake);
    elevatorMotor2.setIdleMode(IdleMode.kBrake);

    elevatorMotor2.follow(elevatorMotor1);

  }

  public void setElevatorPower(double power) {
    elevatorMotor1.set(power);
    SmartDashboard.putNumber("Elevator", elevatorEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
