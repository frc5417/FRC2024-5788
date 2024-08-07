// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final CANSparkMax shooterMotor1;
  private final CANSparkMax shooterMotor2;
  private final CANSparkMax shooterIndex;

  private double[] shooterSpeed_pub = {0.0, 0.0};
  public Supplier<double[]> shooterSpeed_supp = ()->shooterSpeed_pub;

  private double indexSpeed_pub = 0.0;
  public DoubleSupplier indexSpeed_supp = ()->indexSpeed_pub;

  public Shooter() {
    shooterMotor1 = new CANSparkMax(Constants.ManipulatorConstants.shooterMaster, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(Constants.ManipulatorConstants.shooterSlave, MotorType.kBrushless);
    shooterIndex = new CANSparkMax(Constants.ManipulatorConstants.shooterIntake, MotorType.kBrushless);

    shooterMotor1.setInverted(true);
    shooterMotor2.setInverted(true);

    shooterIndex.setInverted(false);
  }

  public void setShooterPower(double power) {
    shooterMotor1.set(power);
    shooterMotor2.set(power);
  }

  public void setDifferentialShooterPower(double power1, double power2) {
    shooterMotor1.set(power1);
    shooterMotor2.set(power2);
  }

  public void setShooterIntake(double speed) {
    shooterIndex.set(speed);
  }

  public Command ShootDaNote(double speed) {
    return run(() -> setShooterPower(speed));
  }

  public Command IndexDaNote(double speed) {
    return run(() -> setShooterIntake(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterSpeed_pub[0] = shooterMotor1.get();
    shooterSpeed_pub[1] = shooterMotor2.get();
    indexSpeed_pub = shooterIndex.get();
  }
}
