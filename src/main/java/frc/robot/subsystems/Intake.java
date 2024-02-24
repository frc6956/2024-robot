// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkFlex upperIntakeMotor;
  CANSparkFlex lowerIntakeMotor;

  RelativeEncoder upperEncoder;
  RelativeEncoder lowerEncoder;

  DigitalInput inputBreak;


  public Intake() {
    inputBreak = new DigitalInput(IntakeConstants.intakeBreakID);

    upperIntakeMotor = new CANSparkFlex(IntakeConstants.upIntakeID, MotorType.kBrushless);
    upperIntakeMotor.restoreFactoryDefaults();
    upperIntakeMotor.setInverted(IntakeConstants.upInvert);
    upperIntakeMotor.burnFlash();

    lowerIntakeMotor = new CANSparkFlex(IntakeConstants.lowIntakeID, MotorType.kBrushless);
    lowerIntakeMotor.restoreFactoryDefaults();
    lowerIntakeMotor.setInverted(IntakeConstants.lowInvert);
    lowerIntakeMotor.burnFlash();

    upperEncoder = upperIntakeMotor.getEncoder();
    lowerEncoder = lowerIntakeMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Has Note", hasNote());
  }

  public void setSpeed(double speed){
    upperIntakeMotor.set(speed);
    lowerIntakeMotor.set(speed);
  }

  public int getRPM(){
    int rpm = (int)(Math.abs(upperEncoder.getVelocity()) + Math.abs(lowerEncoder.getVelocity()))/2;
    return rpm;  
  }

  public void stop(){
    upperIntakeMotor.set(0);
    lowerIntakeMotor.set(0);
  }

  public boolean hasNote(){
    return inputBreak.get();
  }
}
