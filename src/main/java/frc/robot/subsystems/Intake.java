// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkFlex upperIntakeMotor;
  CANSparkFlex lowerIntakeMotor;


  public Intake() {

    upperIntakeMotor = new CANSparkFlex(IntakeConstants.upIntakeID, MotorType.kBrushless);
    upperIntakeMotor.setInverted(IntakeConstants.upInvert);

    lowerIntakeMotor = new CANSparkFlex(IntakeConstants.lowIntakeID, MotorType.kBrushless);
    lowerIntakeMotor.setInverted(IntakeConstants.lowInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    upperIntakeMotor.set(speed);
    lowerIntakeMotor.set(speed);
  }

  public void stop(){
    upperIntakeMotor.set(0);
    lowerIntakeMotor.set(0);
  }
}
