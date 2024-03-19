// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private double m_rainbowFirstPixelHue = 0;
  int count = 0;
  boolean toggled = false;
  
  private Panel leftPanel = new Panel(0, LEDConstants.panelWidth, LEDConstants.panelHeight);
  private Panel rightPanel = new Panel(leftPanel.getEnd(), LEDConstants.panelWidth, LEDConstants.panelHeight);

  public LEDs() {
    //PWM port 1 on the Rio
    m_led = new AddressableLED(0);

    //sets the length of the LEDs
    m_ledBuffer = new AddressableLEDBuffer(getLength());

    setUpLight();

    count = 0;    
  }

  public void setUpLight(){
    //defines the length of the leds
    //setting the length is expensive so only call it once
    m_led.setLength(m_ledBuffer.getLength());

    //sets the LED data and starts the signal
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public int getLength(){
    return rightPanel.getEnd();
  }

  public int[] getAllLEDs(){
    return new int[] {0, getLength()};
  }

  public int[] getPanelsRange(){
    return new int[] {getLeftPanelRange()[0], getRightPanelRange()[1]};
  }

  public int[] getLeftPanelRange(){
    return new int[] {leftPanel.getStart(), leftPanel.getEnd()};
  }

  public int[] getRightPanelRange(){
    return new int[] {rightPanel.getStart(), rightPanel.getEnd()};
  }

  public Panel getLeftPanel(){
    return leftPanel;
  }

  public Panel getRightPanel(){
    return rightPanel;
  }

  public void setAllColor(int[] color){
    setAllPanelColor(color);
  }

  public void setAllPanelColor(int[] color){
    setPanelColor(leftPanel, color);
    setPanelColor(rightPanel, color);
  }

  public void setColor(int[] startEnd, int[] color){
    for (int i = startEnd[0]; i < startEnd[1]; i++){
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
    m_led.setData(m_ledBuffer);
  }

  private void setPanelColor(Panel panel, int[] color){
    m_led.setData(panel.setAllColor(m_ledBuffer, color));
  }

  public void setAllOff(){
    setAllColor(new int[] {0, 0, 0});
  }

  public void setRainbow(int[] startEnd){
    for (int i = startEnd[0]; i < startEnd[1]; i++){
      final int hue = ((int)(m_rainbowFirstPixelHue + (i * 180 / startEnd[1]))) % 180;
      m_ledBuffer.setHSV(i, hue, 150, 200);
    }
    m_rainbowFirstPixelHue += 0.5;

    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  public void dimRainbow(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180;
      if (i %2 == 0){
        
        m_ledBuffer.setHSV(i, hue, 250, 10);
      } else {
        m_ledBuffer.setHSV(i, hue, 250, 140);
      }

      m_rainbowFirstPixelHue += 0.015;

      m_rainbowFirstPixelHue %= 180;

    }

    m_led.setData(m_ledBuffer);
  }

  public void setShape(Panel panel, Shape shape){
    m_led.setData(panel.setShape(m_ledBuffer, shape.get()));
  }

  public int[][][] getMirrored(int[][][] state){
    int[][][] leds = state;
    for (int row = 0; row < leds.length; row++){
      for (int col = 0; col < leds[0].length; col++){
        int[] tempColor = leds[row][col];
        leds[row][col] = leds[row][leds[row].length - col - 1];
        leds[row][leds[row].length - col - 1] = tempColor;
      }
    }
    return leds;
  }

  public void togglePanel(Panel firstPanel, Panel secondPanel, int[][][] state){
    count++;
    if (count == 1){
      if (toggled){
        firstPanel.setShape(m_ledBuffer, state);
        setPanelColor(secondPanel, LEDConstants.n);
        toggled = false;
      } else {
        secondPanel.setShape(m_ledBuffer, state);
        setPanelColor(firstPanel, LEDConstants.n);
        toggled = true;
      }
      state = getMirrored(state);
    } else if (count > 100){
      count = 0;
    }
  }

}
