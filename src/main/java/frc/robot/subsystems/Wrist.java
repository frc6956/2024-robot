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
  PIDController angleUpController;
  PIDController angleUpGravController;

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
    angleUpController = new PIDController(WristConstants.wristPUP, WristConstants.wristI, WristConstants.wristD);
    angleUpGravController = new PIDController(WristConstants.wristPUPGrav, WristConstants.wristI, WristConstants.wristD);

    angleController.setTolerance(3);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);

    rightMotor.setInverted(WristConstants.rightInvert);
    leftMotor.setInverted(WristConstants.leftInvert);

    
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
    target = setpoint;
    double output;
    /*if (getDegrees() - setpoint < 0){
      output = angleUpController.calculate(getDegrees(), setpoint);
    } else {*/
      output = angleController.calculate(getDegrees(), setpoint);
    //}
    
    if (output > WristConstants.MaxRotateUpSpeed){
      output = WristConstants.MaxRotateUpSpeed;
    } else if (output < WristConstants.MaxRotateSpeed){
      output = WristConstants.MaxRotateSpeed;
    } else if (Math.abs(output) < 0.003){
      output = 0;
    }
    leftMotor.set(output);
    rightMotor.set(output); 
  }

  public double getTarget(){
    return target;
  }

  public void setTarget(double rotation){
    this.target = rotation;
  }

  public boolean onTarget() {
    return angleController.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", getDegrees());

  }
}
