// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

/** Add your docs here. */
public class LEDConstants {
    public static final int panelWidth = 16;
    public static final int panelHeight = 16;

    public static final int frontUnderGlowLength = 14;
    public static final int leftUnderGlowLength = 14;
    public static final int backUnderGlowLength = 14;
    public static final int rightUnderGlowLength = 14;

    public static final int intakeGlow = 20;

    public static final int[] green = {0, 50, 0};
    public static final int[] blue = {0, 0, 50};
    public static final int[] red = {50, 0, 0};

    public static final int[] n = {0, 0, 0};
    public static final int[] G = green;

    public static final Shape shamrock = new Shape(new int[][][]{
        {n, n, red, red, red, G, G, n, n, G, G, n, n, n, n, n}, // 1
        {n, n, n, n, G, G, G, G, G, G, G, G, n, n, n, n}, // 2
        {n, n, n, n, G, G, G, G, G, G, G, G, n, n, n, n}, // 3
        {n, n, n, n, G, G, G, G, G, G, G, G, n, n, n, n}, // 4
        {n, n, n, n, n, G, G, G, G, G, G, n, n, n, n, n}, // 5
        {n, n, G, G, G, n, G, G, G, G, n, G, G, G, n, n}, // 6
        {n, G, G, G, G, G, G, G, G, G, G, G, G, G, G, n}, // 7
        {n, G, G, G, G, G, G, G, G, G, G, G, G, G, G, n}, // 8
        {n, n, G, G, G, G, G, G, G, G, G, G, G, G, n, n}, // 9
        {n, n, G, G, G, red, G, G, G, G, G, G, G, G, n, n}, // 10
        {n, G, G, G, G, G, G, G, G, G, G, G, G, G, G, n}, // 11
        {n, G, G, G, G, G, n, G, G, n, G, G, G, G, G, n}, // 12
        {n, n, G, G, G, n, n, G, G, n, n, G, G, G, n, n}, // 13
        {n, n, n, n, n, n, n, n, G, G, n, n, n, n, n, n}, // 14
        {n, n, n, n, n, n, n, n, n, G, G, n, n, n, n, n}, // 15
        {n, n, n, n, n, n, n, n, n, n, n, G, n, n, n, n}  // 16
    });

}
