// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//*****************************************************************IMPLEMENT NETWORKTABLES AND SMART DASHBOARD*************************************/
package frc.robot.sensors;


import java.util.function.BiConsumer;
import java.io.IOException;
import java.nio.file.ProviderMismatchException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera cam;
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator poseEstimator;

  public PhotonVision() throws IOException{
    cam = new PhotonCamera(VisionConstants.camName);
    try{
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch(IOException IOE){
      IOE.printStackTrace();
    }

    poseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, VisionConstants.RobotToCam);
  }

  @Override
  public void periodic() {
    
  }

  public PhotonPipelineResult getLatestResult(){
    return cam.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getVisionPoseEstimationResult(){
    return poseEstimator.update();
  }

  public EstimatedRobotPose ifExistGetEstimatedPose(){
    if (getVisionPoseEstimationResult().isPresent()){
      return getVisionPoseEstimationResult().get();
    } else return null;
  }


  //double check this
  public PhotonTrackedTarget getBestTarget(){
    return getLatestResult().getBestTarget();
  }

  public Transform3d getCamToTarget(){
    return getBestTarget().getBestCameraToTarget();
  }

  public List<Integer> getAprilTagIDs(){
    List<PhotonTrackedTarget> targets = getLatestResult().getTargets();
    List<Integer> tagIDs = new ArrayList<>();
    targets.forEach(target -> tagIDs.add(target.getFiducialId()));

    return tagIDs;
  }

  public boolean containsID(Integer ID){
    return getAprilTagIDs().contains(ID);
  }

  public boolean hasAprilTag(){
    for (Integer id : getAprilTagIDs()) {
      if (containsID(id)){
        return true;
      }
    }
    return false;
  }

  public boolean containsSource(){
    return
    containsID(VisionConstants.AprilTagIDs.BlueSourceDriverStationClose.getID()) ||
    containsID(VisionConstants.AprilTagIDs.BlueSourceDriverStationFar.getID()) ||
    containsID(VisionConstants.AprilTagIDs.RedSourceDriverStationClose.getID()) ||
    containsID(VisionConstants.AprilTagIDs.RedSourceDriverStationFar.getID());
  }

  public boolean containsAmp(){
    return
    containsID(VisionConstants.AprilTagIDs.RedAmp.getID()) ||
    containsID(VisionConstants.AprilTagIDs.BlueAmp.getID());
  }

  public boolean containsSpeakerCenter(){
    return
    containsID(VisionConstants.AprilTagIDs.RedSpeakerCenter.getID()) ||
    containsID(VisionConstants.AprilTagIDs.BlueSpeakerCenter.getID());
  }

  public boolean bestTargetIsSpeaker(){
    return
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.BlueSpeakerCenter.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.BlueSpeakerSide.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.RedSpeakerCenter.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.RedSpeakerSide.getID();
  }

  public boolean bestTargetIsCenterSpeaker(){
    return 
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.BlueSpeakerCenter.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.RedSpeakerCenter.getID();
  }

  public boolean bestTargetIsSource(){
    return
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.BlueSourceDriverStationClose.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.BlueSourceDriverStationFar.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.RedSourceDriverStationClose.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.RedSourceDriverStationFar.getID();
  }

  public boolean bestTargetIsAmp(){
    return
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.BlueAmp.getID() ||
    getBestTarget().getFiducialId() == VisionConstants.AprilTagIDs.RedAmp.getID();
  }
}
