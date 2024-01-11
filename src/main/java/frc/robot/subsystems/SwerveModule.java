// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  //private final RelativeEncoder driveEncoder;
  //private final RelativeEncoder turningEncoder;

  //private final PIDController turningPidController;


  public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, 
  boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();
    

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
