// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final CANSparkMax shooterMotor1;
  private final CANSparkMax shooterMotor2;
  private final CANSparkMax shooterIndex;

  public Shooter() {
    shooterMotor1 = new CANSparkMax(Constants.ManipulatorConstants.elevatorMaster, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(Constants.ManipulatorConstants.elevatorSlave, MotorType.kBrushless);
    shooterIndex = new CANSparkMax(Constants.ManipulatorConstants.shooterIntake, MotorType.kBrushless);

    shooterMotor1.setInverted(false);
    shooterMotor2.setInverted(true);

    shooterIndex.setInverted(false);
  }

  public void setShooterPower(double power) {
    shooterMotor1.set(power);
    shooterMotor2.set(power);
  }

  public void setShooterIntake(double speed) {
    shooterIndex.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
