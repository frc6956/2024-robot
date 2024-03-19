// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shape extends SubsystemBase {
  /** Creates a new Shape. */
  private int[][][] image;
  private int[][][] initialShape;
  public Shape(int[][][] initShape) {
    initialShape = initShape;
    image = setUpShape(initShape);
  }

  public int[][][] get(){
    return image;
  }

  public static int[][][] setUpShape(int[][][] shape){
    //need to reverse the 15th row
    int[][] fifteen = shape[14];
    for (int i = 0; i < fifteen.length/2; i++){
        int[] temp = fifteen[i];
        fifteen[i] = fifteen[fifteen.length - i - 1];
        fifteen[fifteen.length - 1 - i] = temp;
    }
    shape[14] = fifteen;

    
    //rotate 90 degrees clockwise
    int N = shape.length;
    for (int i = 0; i < N / 2; i++) {
      for (int j = i; j < N - i - 1; j++) {

          // Swap elements of each cycle
          // in clockwise direction
          int[] temp = shape[i][j];
          shape[i][j] = shape[N - 1 - j][i];
          shape[N - 1 - j][i] = shape[N - 1 - i][N - 1 - j];
          shape[N - 1 - i][N - 1 - j] = shape[j][N - 1 - i];
          shape[j][N - 1 - i] = temp;
      }
    }
    return shape;
  }

  
}
