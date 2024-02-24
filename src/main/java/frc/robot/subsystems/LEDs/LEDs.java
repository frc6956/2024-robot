// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_Constants;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private Panel panel = new Panel(LED_Constants.panelWidth, LED_Constants.panelHeight);

  private Strip intakeGlow = new Strip(LED_Constants.intakeGlow);

  private Strip frontGlow = new Strip(LED_Constants.frontUnderGlowLength);
  private Strip backGlow = new Strip(LED_Constants.backUnderGlowLength);
  private Strip leftGlow = new Strip(LED_Constants.leftUnderGlowLength);
  private Strip rightGlow = new Strip(LED_Constants.rightUnderGlowLength);

  Strip[] strips = new Strip[] {
      frontGlow,
      backGlow,
      leftGlow,
      rightGlow,
      intakeGlow
  };

  public LEDs() {
    //PWM port 1 on the Rio
    m_led = new AddressableLED(1);

    //sets the length of the LEDs
    m_ledBuffer = new AddressableLEDBuffer(getLength());

    setUpLight();

    
  }

  public void setUpLight(){
    //defines the length of the leds
    //setting the length is expensive so only call it once
    m_led.setLength(m_ledBuffer.getLength());

    //sets the LED data and starts the signal
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  //returns the length of the total amount of LEDs on the robot

  public int getLength(){
    int length = panel.getLength();
    return length;
  }

}
