// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  /** Creates a new Intake. */

  DutyCycleEncoder angleEncoder;

  CANSparkMax angleMotor;

  PIDController angleController;

  double target = WristConstants.STOW;

  public Wrist() {
    angleEncoder = new DutyCycleEncoder(WristConstants.wristPort);

    angleMotor = new CANSparkMax(WristConstants.wristID, MotorType.kBrushless);
    angleMotor.enableVoltageCompensation(Constants.voltageComp);

    angleController = new PIDController(WristConstants.wristP, WristConstants.wristI, WristConstants.wristD);

    
  }

  
  public void setPosition(double rotation){

    if (rotation < getDegrees() && getDegrees() < WristConstants.PICKUP) {
            rotation = WristConstants.PICKUP;
    } else if (rotation > getDegrees() && getDegrees() > WristConstants.STOW) {
            rotation = WristConstants.STOW;
    }
    
    target = rotation;
    
    angleMotor.set(angleController.calculate(getDegrees(), rotation));
  }


  public double getDegrees(){
    double degrees = 360*(angleEncoder.getAbsolutePosition()) - WristConstants.offset;

    if (degrees > 360){
      degrees -=360;
    } else if (degrees < 0){
      degrees +=360;
    }
    return degrees;
  }


  public void holdWrist(){
    angleMotor.set(angleController.calculate(getDegrees(), target));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Through Bore Encoder", getDegrees());
  }
}
