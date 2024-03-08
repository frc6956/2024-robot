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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class PhotonVision extends SubsystemBase {
  private String camName;
  private PhotonCamera cam;
  private boolean hasTargets;
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator photonEstimator;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private int tagID = -1;
  private Pose3d robotPose = new Pose3d();
  private Swerve swerve;
  private PhotonPipelineResult result;
  private double lastEstTimestamp = 0;

  public PhotonVision (Swerve swerve) {
    cam = new PhotonCamera(VisionConstants.camName);
    this.swerve = swerve;
    hasTargets = false;

    try {
        fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
    
        e.printStackTrace();
      }

    photonEstimator = new PhotonPoseEstimator(
      fieldLayout, 
      PoseStrategy.AVERAGE_BEST_TARGETS, 
      cam,
      VisionConstants.RobotToCam);
    //photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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

    var visionEst = getEstimatedGlobalPose();
    SmartDashboard.putBoolean("Estimator is Present", visionEst.isPresent());
    visionEst.ifPresent(
      est -> {
        var estPose = est.estimatedPose.toPose2d();
        var estStdDevs = getEstimatedStdDevs(estPose);
        swerve.addVisionMeasurement(est.estimatedPose, est.timestampSeconds, estStdDevs);
      }
    );
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    var visionEstimation = photonEstimator.update();
    double latesttimeStamp = cam.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latesttimeStamp - lastEstTimestamp) > 1e-6;
    if (newResult){lastEstTimestamp = latesttimeStamp;}
    return visionEstimation;
  }

  public Matrix<N3, N1> getEstimatedStdDevs(Pose2d estimatedPose){
    var estStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    var t = cam.getLatestResult().getTargets();
    int numTags = 0;
    double avgDistance = 0;
    for (var tar : t){
      var tagPose = photonEstimator.getFieldTags().getTagPose(tar.getFiducialId());
      if (tagPose.isEmpty()){
        continue;
      }
      numTags++;
      avgDistance +=
        tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0){
      return estStdDevs;
    }

    avgDistance /= numTags;
    if (numTags > 1){
      estStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }
    if (numTags == 1 && avgDistance > 4){
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else estStdDevs = estStdDevs.times(1 + (avgDistance * avgDistance / 30));

    return estStdDevs;
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
