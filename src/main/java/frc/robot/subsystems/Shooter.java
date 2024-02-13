// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkFlex leftShooter;
  CANSparkFlex rightShooter;
  public Shooter() {

    leftShooter = new CANSparkFlex(ShooterConstants.leftShooterID, MotorType.kBrushless);
    rightShooter = new CANSparkFlex(ShooterConstants.rightShooterID, MotorType.kBrushless);

  }

  public void setSpeed(double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void stop(){
    leftShooter.set(0);
    rightShooter.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
