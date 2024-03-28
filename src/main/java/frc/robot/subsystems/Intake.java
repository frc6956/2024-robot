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
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkFlex upperIntakeMotor;
  CANSparkFlex lowerIntakeMotor;

  RelativeEncoder upperEncoder;
  RelativeEncoder lowerEncoder;

  DigitalInput inputBreak;

  /**
   * The Intake class represents the intake subsystem of the robot.
   * It controls the intake motors and encoders.
   */
  public Intake() {
    inputBreak = new DigitalInput(IntakeConstants.intakeBreakID);

    upperIntakeMotor = new CANSparkFlex(IntakeConstants.upIntakeID, MotorType.kBrushless);
    upperIntakeMotor.restoreFactoryDefaults();
    upperIntakeMotor.setInverted(IntakeConstants.upInvert);
    upperIntakeMotor.enableVoltageCompensation(Constants.voltageComp);
    upperIntakeMotor.setSmartCurrentLimit(60);

    lowerIntakeMotor = new CANSparkFlex(IntakeConstants.lowIntakeID, MotorType.kBrushless);
    lowerIntakeMotor.restoreFactoryDefaults();
    lowerIntakeMotor.enableVoltageCompensation(Constants.voltageComp);
    lowerIntakeMotor.setInverted(IntakeConstants.lowInvert);
    lowerIntakeMotor.setSmartCurrentLimit(60);

    upperEncoder = upperIntakeMotor.getEncoder();
    lowerEncoder = lowerIntakeMotor.getEncoder();
  }

  /**
   * This method is called periodically by the scheduler.
   * It updates the SmartDashboard with the boolean value indicating whether the
   * intake has a note.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Has Note", hasNote());
  }

  /**
   * Sets the speed of the intake motors.
   * 
   * @param speed the speed to set the intake motors to
   */
  public void setSpeed(double speed) {
    upperIntakeMotor.set(speed);
    lowerIntakeMotor.set(speed);
  }

  /**
   * Returns the average RPM (Rotations Per Minute) of the intake motors.
   *
   * @return The average RPM of the intake motors.
   */
  public int getRPM() {
    int rpm = (int) (Math.abs(upperEncoder.getVelocity()) + Math.abs(lowerEncoder.getVelocity())) / 2;
    return rpm;
  }

  /**
   * Stops the intake motors.
   */
  public void stop() {
    upperIntakeMotor.set(0);
    lowerIntakeMotor.set(0);
  }

  /**
   * Checks if the intake has detected a note.
   * 
   * @return true if a note is detected, false otherwise
   */
  public boolean hasNote() {
    return inputBreak.get();
  }
}
