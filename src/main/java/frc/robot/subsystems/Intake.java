// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final CANSparkFlex leftShooter;
  private final CANSparkFlex rightShooter;

  private final CANSparkMax leftElev;
  private final CANSparkMax rightElev;

  private final Encoder encoder;

  public Intake() {

    leftShooter = new CANSparkFlex(Constants.NeoMotorConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
    rightShooter = new CANSparkFlex(Constants.NeoMotorConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

    leftElev = new CANSparkMax(Constants.NeoMotorConstants.SHOOTER_ELEV_LEFT_ID, MotorType.kBrushless);
    rightElev =new CANSparkMax(Constants.NeoMotorConstants.SHOOTER_ELEV_RIGHT_ID, MotorType.kBrushless);

    encoder = new Encoder(0, 1);

    leftShooter.setInverted(true);

    leftElev.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double speed){
    // add which shooter motors are inverted
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void intakeHP(double speed){
    // add which shooter motors are inverted
    //Intake will be reversed 
    leftShooter.set(speed);
    rightShooter.set(speed);
  }


  public void setAngle(double angle){
    // Sets shooter to input angle

  }

  public void setAngleDynamic(double distance){
    Math.atan(Constants/distance);

  }

  public void resetAngle(){
    // Sets shooter angle to the 55 degree position (0 degrees)
  }

}
