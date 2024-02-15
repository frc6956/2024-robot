// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

public class AlignToTagPhotonVision extends Command {
  /** Creates a new AlignToTagPhotonVision. */
  Swerve swerve;
  PhotonVision vision;
  Transform3d camToTarget;

  double xError;
  double yError;
  double thetaError;
  boolean xFinished;
  boolean yFinished;
  boolean thetaFinished;

  public AlignToTagPhotonVision(Swerve swerve, PhotonVision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camToTarget = vision.getCamToTarget();
    xError = CommandConstants.xGoal - camToTarget.getX();
    yError = CommandConstants.yGoal - camToTarget.getY();
    thetaError = camToTarget.getRotation().getAngle();

    swerve.drive(
      new Translation2d(-xError, -yError).times(DriveConstants.MaxSpeed), 
      thetaError, 
      false, true, false, false);

    if (Math.abs(xError) < CommandConstants.errorTolerence){
      xFinished = true;
    } else xFinished = false;
    if (Math.abs(yError) < CommandConstants.errorTolerence){
      yFinished = true;
    } else yFinished = false;
    if (Math.abs(thetaError) < CommandConstants.errorTolerence){
      thetaFinished = true;
    } else thetaFinished = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xFinished && yFinished && thetaFinished;
  }
}
