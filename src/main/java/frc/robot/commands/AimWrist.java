// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.opencv.photo.Photo;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.sensors.PhotonCam;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class AimWrist extends Command {
  /** Creates a new AimWrist. */
  private Wrist wrist;
  private Swerve swerve;
  private PhotonVision photonVision;
  private boolean isBlue;
  public AimWrist(Wrist wrist, Swerve swerve, PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.photonVision = photonVision;
    this.swerve = swerve;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      isBlue = true;
    } else isBlue = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose;
    if (isBlue){
      targetPose = photonVision.getTagPose(7).get().toPose2d();
    } else {
      targetPose = photonVision.getTagPose(4).get().toPose2d();
    }
    double distanceToTarget = PhotonUtils.getDistanceToPose(swerve.getPose(), targetPose);

    double targetWrist = Math.toDegrees(
          Math.atan((
            VisionConstants.SPEAKER_SCORE_HEIGHT - VisionConstants.WRIST_AXLE_HEIGHT)
            / distanceToTarget));
        
        wrist.holdWrist(getShootAngle(targetWrist));
  }

  public double getShootAngle(double angle){
    double shootAngle = angle;
    Math.min(shootAngle, WristConstants.SUBWOOFER);
    return shootAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }
}
