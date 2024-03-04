// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Strip extends SubsystemBase {
  /** Creates a new Strip. */
  private int length;
  private final int start;
  private final int end;

  private int[][] leds;

  public Strip(int start, int length) {
    this.length = length;
    leds = new int[length][3];
    this.start = start;
    this.end = start + getLength();
  }

  public int getLength(){
    return length;
  }

  public int getStart(){
    return start;
  }

  public int getEnd(){
    return end;
  }

  public int[][] getLEDs(){
    return leds;
  }

  public int getIndex(int index){
    return index + start;
  }

  public AddressableLEDBuffer setAllColor(AddressableLEDBuffer m_ledBuffer, int[] color){
    for (int i = 0; i < getLength(); i++){
      leds[i] = color;
      m_ledBuffer.setRGB(getIndex(i), color[0], color[1], color[2]);
    }
    return m_ledBuffer;
  }
}
