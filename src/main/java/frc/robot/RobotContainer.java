// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;

public class RobotContainer {


  /* Controllers */
  private final XboxController driver = new XboxController(OperatorConstants.driverPort);
  private final XboxController operator = new XboxController(OperatorConstants.operatorPort);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton isEvading = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton isLocked = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Operator Buttons */

  

  /* Subsystems */
  private final Swerve swerve = new Swerve();


  /* Auton Chooser */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    swerve.setDefaultCommand(
      new SwerveDrive(
        swerve, 
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis), 
        () -> false,
        () -> isEvading.getAsBoolean(),
        () -> isLocked.getAsBoolean()
      )
    );

    /* Pathplanner Named Commands */
      //NamedCommands.registerCommand("Intake", new IntakeNote(m_intake, m_robotState));


    /* Autos */
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    configureBindings();
  }

  private void configureBindings() {

    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void printValues(){
    /* Robot Position */
    SmartDashboard.putString("Robot Pose2d", swerve.getPose().getTranslation().toString());
    SmartDashboard.putNumber("Robot Yaw", swerve.getYaw());
    SmartDashboard.putNumber("Robot Pitch", swerve.getPitch());
    SmartDashboard.putNumber("Robot Roll", swerve.getRoll());
  }
}
