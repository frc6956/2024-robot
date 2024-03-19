// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Panel extends SubsystemBase {
  /** Creates a new Panel. */
  private int width;
  private int height;
  private final int start;
  private final int end;
  private int[][][] leds;

  public Panel(int start, int width, int height) {
    this.width = width;
    this.height = height;
    leds = new int[width][height][3];
    this.start = start;
    this.end = start + getLength();
  }

  public int getLength(){
    return width * height;
  }

  public int getStart(){
    return start;
  }

  public int getEnd(){
    return end;
  }

  public int[][][] getLEDs(){
    return leds;
  }

  public int getIndex(int row, int col){
    int index = start;
    index += (row * 16) + col;
    return index;
  }

  public AddressableLEDBuffer writeLEDs(AddressableLEDBuffer m_ledBuffer){
    for (int row = 0; row < leds.length; row++){
      for (int col = 0; col < leds[0].length; col++){
        m_ledBuffer.setRGB(getIndex(row, col), leds[row][col][0], leds[row][col][1], leds[row][col][2]);
      }
    }
    return m_ledBuffer;
  }

  public AddressableLEDBuffer setAllColor(AddressableLEDBuffer m_ledBuffer, int[] color){
    for (int i = start; i < end; i++){
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
    return m_ledBuffer;
  }

  public AddressableLEDBuffer setShape(AddressableLEDBuffer m_ledBuffer, int[][][] shape){
    for (int i = 0; i < shape.length; i++){
      for (int j = 0; j < shape[0].length; j++){
        leds[i][j] = shape[i][j];
      }
    }
    return writeLEDs(m_ledBuffer);
  }

  public AddressableLEDBuffer mirror(AddressableLEDBuffer m_ledBuffer, int[][][]mirror){
    leds = mirror;
    for (int row = 0; row < leds.length; row++){
      for (int col = 0; col < leds[0].length / 2; col++){
        int[] tempColor = leds[row][col];
        leds[row][col] = leds[row][leds[row].length - col - 1];
        leds[row][leds[row].length - col - 1] = tempColor;
      }
    }
    return writeLEDs(m_ledBuffer);
  }

}
