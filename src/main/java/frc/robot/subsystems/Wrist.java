// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
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

  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  PIDController angleController;

  double target = WristConstants.STOW;

  public Wrist() {
    angleEncoder = new DutyCycleEncoder(WristConstants.wristPort);

    leftMotor = new CANSparkMax(WristConstants.wristID, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    leftMotor.enableVoltageCompensation(Constants.voltageComp);

    rightMotor = new CANSparkMax(WristConstants.wrist2ID, MotorType.kBrushless);
    rightMotor.restoreFactoryDefaults();
    rightMotor.enableVoltageCompensation(Constants.voltageComp);

    angleController = new PIDController(WristConstants.wristP, WristConstants.wristI, WristConstants.wristD);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);

    rightMotor.setInverted(WristConstants.rightInvert);
    leftMotor.setInverted(WristConstants.leftInvert);

    
  }

  
  public void setPosition(double rotation){

    if (rotation < getDegrees() && getDegrees() < WristConstants.PICKUP) {
            rotation = WristConstants.PICKUP;
    } else if (rotation > getDegrees() && getDegrees() > WristConstants.STOW) {
            rotation = WristConstants.STOW;
    }
    
    target = rotation;
    
    
    leftMotor.set(angleController.calculate(getDegrees(), rotation));
    rightMotor.set(angleController.calculate(getDegrees(), rotation));
  }

  public void setOutput(double output){
    leftMotor.set(output);
    rightMotor.set(output);
  }

  public void stop(){
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void setSpeed(double speed){
    if (speed > 0 && getDegrees() > WristConstants.STOW){
      stop();
    } else if (speed < 0 && getDegrees() < WristConstants.PICKUP){
      stop();
    }
    leftMotor.set(speed);
    rightMotor.set(speed);
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


  public void holdWrist(double setpoint){
    double output = angleController.calculate(getDegrees(), setpoint);
    if (Math.abs(output) > 0.1){
      output = Math.copySign(0.05, output);
    } else if (Math.abs(output) < 0.005){
      output=0;
    }

    leftMotor.set(output);
    rightMotor.set(output); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", getDegrees());

  }
}
