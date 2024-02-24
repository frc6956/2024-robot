// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax leftClimber;
  CANSparkMax rightClimber;

  public Climber() {
    leftClimber = new CANSparkMax(ClimberConstants.leftMotorID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(ClimberConstants.rightMotorID, MotorType.kBrushless);

    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimber.setInverted(ClimberConstants.leftInvert);
    rightClimber.setInverted(ClimberConstants.rightInvert);

    leftClimber.burnFlash();
    rightClimber.burnFlash();
  }

  public void stop(){
    leftClimber.set(0);
    rightClimber.set(0);
  }

  public void extend(){
    leftClimber.set(ClimberConstants.climbSpeed);
    rightClimber.set(ClimberConstants.climbSpeed);
  }

  public void retract(){
    leftClimber.set(-ClimberConstants.climbSpeed);
    rightClimber.set(-ClimberConstants.climbSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
