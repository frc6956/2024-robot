// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final Pigeon2 gyro = new Pigeon2(0);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(gyro);

  private final XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -driverController.getLeftY(), 
      () -> driverController.getLeftX(), 
      () -> driverController.getRightX(), 
      () -> swerveSubsystem.getIsFieldOrientated()));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}