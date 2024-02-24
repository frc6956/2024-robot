// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Strip extends SubsystemBase {
  /** Creates a new Strip. */
  private int length;

  private int[][] leds;
  public Strip(int length) {
    this.length = length;
    leds = new int[length][3];
  }

  public int getLength(){
    return length;
  }

  public int[][] getLEDs(){
    return leds;
  }
}
