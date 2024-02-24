// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.util.function.BiConsumer;

import org.photonvision.proto.Photon;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AlignToTagPhotonVision;
import frc.robot.commands.HoldWrist;
import frc.robot.commands.SetPosition;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TeleopIntakeFeed;
import frc.robot.subsystems.Wrist;
import frc.robot.sensors.FancyLightVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.*;

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
  //private final JoystickButton alignToTag = new JoystickButton(driver, XboxController.Button.kA.value);

  private final JoystickButton pickup = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton rotateUp = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton rotateDown = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton extakeButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton shoot = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
  private final JoystickButton feed = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton climbOveride = new JoystickButton(operator, XboxController.Button.kLeftStick.value);

  /* Operator Buttons */

  

  /* Subsystems */

  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Wrist wrist = new Wrist();
  private FancyLightVision photonVision;
  private final Swerve swerve = new Swerve();
  private final Feeder feeder = new Feeder();



  /* Auton Chooser */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    try{
      photonVision = new FancyLightVision(swerve::visionPose);
    }    
    catch(IOException e){
      DriverStation.reportWarning("Unable to Initialize vision", e.getStackTrace());
    }

    swerve.setDefaultCommand(
      new SwerveDrive(
        swerve, 
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis), 
        () -> false,
        () -> false,
        () -> isLocked.getAsBoolean()
      )
    );
    
    wrist.setDefaultCommand(
      new HoldWrist(wrist)
    );

    //
    intake.setDefaultCommand(
      new RunCommand(
        () -> intake.stop(),
        intake
      )
    );
    
    feeder.setDefaultCommand(
      new RunCommand(
        () -> feeder.stop(),
        feeder
      )
    );
    //

    intake.setDefaultCommand(
      new TeleopIntakeFeed(
        intake, 
        feeder, 
        IntakeConstants.doNothing)
    );

    climber.setDefaultCommand(
      new RunCommand(
        () -> climber.setSpeed(operator.getLeftY()), 
        climber)
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

    pickup.onTrue(new SetPosition(wrist, WristConstants.PICKUP));



    rotateUp.whileTrue(new RunCommand(() -> wrist.setSpeed(0.1), wrist));

    rotateDown.whileTrue(new RunCommand(() -> wrist.setSpeed(-0.05), wrist));

    //intakeButton.whileTrue(new RunCommand(() -> intake.setSpeed(IntakeConstants.intakeSpeed), intake));
    intakeButton.whileTrue(new TeleopIntakeFeed(intake, feeder, IntakeConstants.doIntake));
    //intakeButton.whileTrue(new RunCommand(() -> feeder.setSpeed(IntakeConstants.intakeSpeed), feeder));

    //extakeButton.whileTrue(new RunCommand(() -> intake.setSpeed(IntakeConstants.extakeSpeed), intake));
    extakeButton.whileTrue(new TeleopIntakeFeed(intake, feeder, IntakeConstants.doExtale));
    //extakeButton.whileTrue(new RunCommand(() -> feeder.setSpeed(IntakeConstants.extakeSpeed), feeder));

    //shoot.whileTrue(new RunCommand(() -> intake.setSpeed(-1), intake));
    feed.whileTrue(new TeleopIntakeFeed(intake, feeder, IntakeConstants.doShoot));
    //feed.whileTrue(new RunCommand(() -> feeder.setSpeed(-1), feeder));

    climbOveride.whileTrue(new RunCommand(() -> climber.overideDown(), climber));
    

    //alignToTag.whileTrue(new AlignToTagPhotonVision(swerve, photonVision));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void printValues(){
    /* Robot Position */
    //SmartDashboard.putData("Swerve", swerve);

    SmartDashboard.putString("Robot Pose2d", swerve.getPose().getTranslation().toString());
    SmartDashboard.putString("Robot Pose2d Rotation", swerve.getPose().getRotation().toString());
    SmartDashboard.putNumber("Robot Yaw", swerve.getYaw());
    SmartDashboard.putNumber("IntakeRPM", intake.getRPM());
    //SmartDashboard.putNumber("Robot Pitch", swerve.getPitch());
    //SmartDashboard.putNumber("Robot Roll", swerve.getRoll());
  }
}
