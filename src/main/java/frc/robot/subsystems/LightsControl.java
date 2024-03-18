// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LightsControl extends SubsystemBase {

  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;


  /** Creates a new Lights. */
  public LightsControl() {
    m_led = new AddressableLED(6);
    m_ledBuffer = new AddressableLEDBuffer(50);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLed(int status) {
    if (status == 0) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 0, 0);
      } 
    } else if (status == 1) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      } 
    } else if (status == 2) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      } 
    } else if (status == 3) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      } 
    } else if (status == 4) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 255, 255);
      } 
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}