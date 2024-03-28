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
  PIDController angleControllerHigh;

  double target = WristConstants.STOW;

  /**
   * The Wrist class represents the wrist subsystem of the robot.
   * It controls the angle of the wrist using PID controllers and
   * manages the motors and encoders associated with the wrist.
   */
  public Wrist() {
    angleEncoder = new DutyCycleEncoder(WristConstants.wristPort);

    leftMotor = new CANSparkMax(WristConstants.wristID, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    leftMotor.enableVoltageCompensation(Constants.voltageComp);

    rightMotor = new CANSparkMax(WristConstants.wrist2ID, MotorType.kBrushless);
    rightMotor.restoreFactoryDefaults();
    rightMotor.enableVoltageCompensation(Constants.voltageComp);

    angleControllerHigh = new PIDController(0.0085, 0, 0);
    angleController = new PIDController(WristConstants.wristP, WristConstants.wristI, WristConstants.wristD);

    angleController.setTolerance(3);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);

    rightMotor.setInverted(WristConstants.rightInvert);
    leftMotor.setInverted(WristConstants.leftInvert);

  }

  /**
   * Sets the output value for the wrist motors.
   * 
   * @param output the desired output value for the wrist motors
   */
  public void setOutput(double output) {
    leftMotor.set(output);
    rightMotor.set(output);
  }

  /**
   * Stops the wrist movement by setting the left and right motors to 0.
   */
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  /**
   * Sets the speed of the wrist motors.
   * If the speed is positive and the wrist angle is greater than the stow angle,
   * the wrist motors will stop.
   * If the speed is negative and the wrist angle is less than the pickup angle,
   * the wrist motors will stop.
   * Otherwise, the wrist motors will be set to the specified speed.
   * 
   * @param speed the speed to set the wrist motors to
   */
  public void setSpeed(double speed) {
    if (speed > 0 && getDegrees() > WristConstants.STOW) {
      stop();
    } else if (speed < 0 && getDegrees() < WristConstants.PICKUP) {
      stop();
    } else {
      leftMotor.set(speed);
      rightMotor.set(speed);
    }
  }

  /**
   * Returns the current angle of the wrist in degrees.
   * 
   * @return The angle of the wrist in degrees.
   */
  public double getDegrees() {
    double degrees = 360 * (angleEncoder.getAbsolutePosition()) - WristConstants.offset;

    if (degrees > 360) {
      degrees -= 360;
    } else if (degrees < 0) {
      degrees += 360;
    }
    return degrees;
  }

  /**
   * Holds the wrist at the specified setpoint.
   * If the setpoint is greater than the stow position, it will be set to the stow
   * position.
   * The output is calculated based on the current wrist angle and the setpoint.
   * If the wrist angle is greater than 120 degrees, a high angle controller is
   * used.
   * Otherwise, a regular angle controller is used.
   * The output is limited to the maximum rotate up speed and the maximum rotate
   * speed.
   * The output is then applied to both the left and right wrist motors.
   * 
   * @param setpoint The desired angle setpoint for the wrist.
   */
  public void holdWrist(double setpoint) {

    if (setpoint > WristConstants.STOW) {
      setpoint = WristConstants.STOW;
    }
    target = setpoint;
    double output;
    /*
     * if (getDegrees() - setpoint < 0){
     * output = angleUpController.calculate(getDegrees(), setpoint);
     * } else {
     */
    if (getDegrees() > 120) {
      output = angleControllerHigh.calculate(getDegrees(), setpoint);
    } else {
      output = angleController.calculate(getDegrees(), setpoint);
    }

    // }

    if (output > WristConstants.MaxRotateUpSpeed) {
      output = WristConstants.MaxRotateUpSpeed;
    } else if (output < WristConstants.MaxRotateSpeed) {
      output = WristConstants.MaxRotateSpeed;
    }

    leftMotor.set(output);
    rightMotor.set(output);
  }

  /**
   * Returns the target value for the wrist.
   *
   * @return the target value for the wrist
   */
  public double getTarget() {
    return target;
  }

  /**
   * Sets the target rotation for the wrist.
   * 
   * @param rotation the target rotation in degrees
   */
  public void setTarget(double rotation) {
    this.target = rotation;
  }

  /**
   * Checks if the wrist is on target.
   * 
   * @return true if the wrist is on target, false otherwise
   */
  public boolean onTarget() {
    return angleController.atSetpoint();
  }

  /**
   * This method is called periodically by the scheduler.
   * It updates the value of the "Wrist Encoder" on the SmartDashboard.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", getDegrees());

  }
}
