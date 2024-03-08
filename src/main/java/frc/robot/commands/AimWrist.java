// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.commands;

import org.opencv.photo.Photo;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.Wrist;

public class AimWrist extends Command {
  /** Creates a new AimWrist. 
  PhotonVision photonVision;
  Wrist wrist;
  public AimWrist(PhotonVision photonVision, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photonVision = photonVision;
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (photonVision.hasAprilTag()){
      PhotonPipelineResult result = photonVision.getLatestResult();
      if (result.hasTargets() /*&& photonVision.hasCenterSpeaker(result) && result != null){
        PhotonTrackedTarget target = photonVision.getBestTarget();//photonVision.getSpeakerCenterTarget(result);
        double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(
          VisionConstants.CAMERA_HEIGHT_METERS, 
          VisionConstants.CENTER_SPEAKER_TOPTAG_HEIGHT, 
          VisionConstants.CAMERA_PITCH_RADIANS, 
          Units.degreesToRadians(target.getPitch())
          );
      System.out.println(distanceToTarget);

        double targetWrist = Math.toDegrees(
          Math.atan((
            VisionConstants.SPEAKER_SCORE_HEIGHT - VisionConstants.WRIST_AXLE_HEIGHT)
            / distanceToTarget));
        
        wrist.holdWrist(getShootAngle(targetWrist));
      } else {
        wrist.stop();
      }
    } else {
      wrist.stop();
    }
  }

  public double getShootAngle(double angle){
    double shootAngle = angle + 143;
    Math.min(shootAngle, WristConstants.SUBWOOFER);
    return shootAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }
}*/
