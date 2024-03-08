package frc.robot.sensors;
import java.io.IOException;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class PhotonVision extends SubsystemBase {
  private String camName;
  private PhotonCamera cam;
  private boolean hasTargets;
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator poseEstimator;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private int tagID = -1;
  private Pose3d robotPose = new Pose3d();
  private Swerve swerve;
  private PhotonPipelineResult result;

  public PhotonVision (Swerve swerve) {
    cam = new PhotonCamera(VisionConstants.camName);
    this.swerve = swerve;
    hasTargets = false;

    try {
        fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
    
        e.printStackTrace();
      }

    poseEstimator = new PhotonPoseEstimator(
      fieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      cam,
      VisionConstants.RobotToCam);
  }

  @Override
  public void periodic(){
    result = cam.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets){
      targets = result.getTargets();
      bestTarget = result.getBestTarget();
      tagID = bestTarget.getFiducialId();
      
    } else {
      tagID = -1;
    }

    Optional<EstimatedRobotPose> poseEstimated = poseEstimator.update(result);
      
    if (poseEstimated.isPresent()){
      filterAndAddVisionPose(poseEstimated.get());
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimation){
    poseEstimator.setReferencePose(prevEstimation);
    return poseEstimator.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    return poseEstimator.update();
  }

  public void filterAndAddVisionPose(EstimatedRobotPose pose){
    Matrix<N3, N1> cov = new Matrix<>(Nat.N3(), Nat.N1());
    /* 
    double distance = 0;
    for (var target : pose.targetsUsed){
      distance += target.getBestCameraToTarget().getTranslation().getNorm() / pose.targetsUsed.size();
    }

    if (pose.targetsUsed.size() > 1){
      // multi tag
      double distance2 = Math.max(Math.pow(distance * 0.4, 2), 0.7);
      cov = VecBuilder.fill(distance2, distance2, 0.9);
    } else {
      double distance2 = Math.pow(distance * 1.2, 2);
      cov = VecBuilder.fill(distance2, distance2, 100);
    }

    if (!DriverStation.isDisabled()){
      if (pose.targetsUsed.size() == 1){
        if (Math.abs(pose.estimatedPose.getZ()) > 1.0 
        || pose.estimatedPose.minus(new Pose3d(
          swerve.getPose()))
          .getTranslation()
          .getNorm() > 1.0 
        || distance > 7.0){
          return;
        }
      }
    }*/
    swerve.addVisionMeasurement(pose.estimatedPose, pose.timestampSeconds, cov);
  }

  public boolean hasTarget(){
    return hasTargets;
  }

  public int getTagID(){
    return tagID;
  }

  public Pose3d getRobotPose(){
    return robotPose;
  }


}
