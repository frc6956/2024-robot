// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnglerConstants;

public class Angler extends SubsystemBase {
  /** Creates a new Angler. */
  CANSparkMax leftAngler;
  CANSparkMax rightAngler;

  DutyCycleEncoder angleEncoder;

  PIDController shootercController;

  double target;
  public Angler() {

    angleEncoder = new DutyCycleEncoder(AnglerConstants.anglerPort);

    leftAngler = new CANSparkMax(AnglerConstants.leftAnglerID, MotorType.kBrushless);
    rightAngler = new CANSparkMax(AnglerConstants.rightAnglerID, MotorType.kBrushless);

    shootercController = new PIDController(AnglerConstants.angleP, AnglerConstants.angleI, AnglerConstants.angleD);
  }

  public void setAngle(double angle){
    if (angle < getDegrees() && getDegrees() < AnglerConstants.MIN) {
            angle = AnglerConstants.MIN;
    } else if (angle > getDegrees() && getDegrees() > AnglerConstants.MAX) {
            angle = AnglerConstants.MAX;
    }
    
    target = angle;
    
    double output = shootercController.calculate(getDegrees(), angle);
    leftAngler.set(output);
    rightAngler.set(output);

  }

  public double getDegrees(){
    double degrees = 360*(angleEncoder.getAbsolutePosition()) - AnglerConstants.anglerOffset;

    if (degrees > 360){
      degrees -=360;
    } else if (degrees < 0){
      degrees +=360;
    }
    return degrees;
  }

  public void stop(){
    rightAngler.set(0);
    rightAngler.set(0);
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
