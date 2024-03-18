// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  private static boolean config;

  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;


  /** Creates a new Lights. */
  public Lights() {
    m_led = new AddressableLED(9); // Port on RoboRIO for the LED... CHANGE accordingly

    m_ledBuffer = new AddressableLEDBuffer(100); // Length of the LED's... CHANGE accordingly
    m_led.setLength(m_ledBuffer.getLength());

  //   for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  //     // Sets the specified LED to the RGB values for red
  //     m_ledBuffer.setRGB(i, 255, 0, 0);
  //  }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public static void configLights(Boolean configStatus) {
    // config = configStatus;

    if (configStatus = true) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 255);
     }
    } else if (configStatus = false) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for Black/Off
        m_ledBuffer.setRGB(i, 0, 0, 0);
     }
    }

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
  }
}
