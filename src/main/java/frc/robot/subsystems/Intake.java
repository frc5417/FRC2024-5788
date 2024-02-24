// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax intakeMotor1;

  public final RelativeEncoder intakeEncoder;

  public Intake() {
    intakeMotor1 = new CANSparkMax(Constants.ManipulatorConstants.intake, MotorType.kBrushless);
    intakeEncoder = intakeMotor1.getEncoder();
    intakeMotor1.setIdleMode(IdleMode.kCoast);
  }

  public void setIntakePower(double power) {
      intakeMotor1.set(power);
      SmartDashboard.putNumber("Intake", intakeEncoder.getPosition());
    }

    public Command setIntakeMotorCommand(double speed) {
      return run(() -> intakeMotor1.set(speed));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
