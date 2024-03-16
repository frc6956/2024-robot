// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.io.IOException;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class AimToSpeaker extends Command {
  /** Creates a new AimToSpeaker. */
  private Swerve swerve;
  private boolean isBlue;
  private AprilTagFieldLayout fieldLayout;
  private boolean isDone = false;
  private PIDController rotController = new PIDController(VisionConstants.visionP, VisionConstants.visionI, VisionConstants.visionD);
  public AimToSpeaker(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
    try {
        fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
    
        e.printStackTrace();
      }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      isBlue = true;
    } else isBlue = false;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose;
    if (isBlue){
      targetPose = fieldLayout.getTagPose(7).get().toPose2d();
    } else {
      targetPose = fieldLayout.getTagPose(4).get().toPose2d();
    }

    //double yawError = PhotonUtils.getYawToPose(swerve.getPose(), targetPose).getDegrees();
    double output = rotController.calculate(swerve.getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
    if (output < 0.01){
      isDone = true;
    }
    swerve.drive(new Translation2d(0, 0), output, false, true, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
