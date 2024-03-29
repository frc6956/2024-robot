// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class TurnAround extends Command {
  /** Creates a new TurnAround. */
  Swerve swerve;
  boolean done = false;
  double target;;
  public TurnAround(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    target = swerve.getYaw() + 180;
    if (target > 180){
      target -= 360;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    try (
            PIDController rotController = new PIDController(
                0.1,
                0,
                0
            )
        ) {
            rotController.enableContinuousInput(DriveConstants.MinAngle, DriveConstants.MaxAngle);

            double rotate = rotController.calculate(swerve.getYaw(), target);
            System.out.println(rotate);

            if (rotate < 0.02){
              rotate = 0;
              done = true;
            }
            swerve.drive(
              new Translation2d(0, 0), 
              -rotate, 
              false, 
              true, 
              false, 
              false);
          }
        }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
