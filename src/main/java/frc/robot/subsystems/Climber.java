// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax leftClimber;
  CANSparkMax rightClimber;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  public Climber() {
    leftClimber = new CANSparkMax(ClimberConstants.leftMotorID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(ClimberConstants.rightMotorID, MotorType.kBrushless);

    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimber.setInverted(ClimberConstants.leftInvert);
    rightClimber.setInverted(ClimberConstants.rightInvert);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();

    resetPostion();
  }

  public void resetPostion(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void stop(){
    leftClimber.set(0);
    rightClimber.set(0);
  }

  public void extend(){
    if (getAverageEncoderPosition() < 280){
      leftClimber.set(ClimberConstants.climbSpeed);
      rightClimber.set(ClimberConstants.climbSpeed);
    } else stop();
  }

  public void retract(){
    if (getAverageEncoderPosition() > 10){
      leftClimber.set(-ClimberConstants.climbSpeed);
      rightClimber.set(-ClimberConstants.climbSpeed);
    } else stop();
    
  }

  public void overideDown(){
    if (getAverageEncoderPosition() < 0){
      resetPostion();
    }
    leftClimber.set(-ClimberConstants.climbSpeed);
    rightClimber.set(-ClimberConstants.climbSpeed);
  }

  public void setSpeed(double speed){
    if (speed > .1){
      extend();
    } else if (speed < -0.1){
      retract();
    } else stop();
  }

  public double getAverageEncoderPosition(){
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Average Climb", getAverageEncoderPosition());
  }
}
