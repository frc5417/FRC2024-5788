// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax intakeMotor1;

  public final RelativeEncoder intakeEncoder;

  // private List<Double> amps = new ArrayList<Double>();
  // private double amps_avg;

  private double intakeSpeed_pub = 0.0;
  public DoubleSupplier intakeSpeed_supp = ()->intakeSpeed_pub;
  private double intakeAmps_pub = 0.0;
  public DoubleSupplier intakeAmps_supp = ()->intakeAmps_pub;

  // public DoubleSupplier intakeSpeedSupplier = ()->intakeMotor1.getAppliedOutput();
  
  public Intake() {
    intakeMotor1 = new CANSparkMax(Constants.ManipulatorConstants.intake, MotorType.kBrushless);
    intakeEncoder = intakeMotor1.getEncoder();
    intakeMotor1.setIdleMode(IdleMode.kCoast);
    intakeMotor1.setSmartCurrentLimit(40);
    // amps.add(0.0);
  }

  public void setIntakePower(double power) {
    intakeMotor1.set(power);
    // SmartDashboard.putNumber("Intake", intakeEncoder.getPosition());
  }

  public Boolean noteInIntake() {
    if (10 >= Constants.ManipulatorConstants.intakeCurrentLimit) {
      return true;
    } else {
      return false;
    }
  }

  public Command IntakeDaNote(double speed) {
    return run(() -> intakeMotor1.set(speed));
  }

  public Command StopIntake() {
    return run(() -> intakeMotor1.set(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeSpeed_pub = intakeMotor1.get();
    intakeAmps_pub = intakeMotor1.getOutputCurrent();
    // if (amps.size() < 60) {
    //   amps.add(intakeMotor1.getOutputCurrent());
    // } else {
    //   amps.remove(0);
    //   amps.add(intakeMotor1.getOutputCurrent());
    // }
    // amps_avg = 0;
    // for (double a : amps) {
    //   amps_avg += a;
    // }
    // amps_avg /= 60;
  }
}
