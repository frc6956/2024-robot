// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Panel extends SubsystemBase {
  /** Creates a new Panel. */
  private int width;
  private int height;
  private int[][][] leds;

  public Panel(int width, int height) {
    this.width = width;
    this.height = height;

    leds = new int[width][height][3];
  }

  public int getLength(){
    return width * height;
  }

  public int[][][] getLEDs(){
    return leds;
  }

}
