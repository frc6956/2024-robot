package frc.robot.sensors;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class PhotonCam extends SubsystemBase {
  private String camName;
  private PhotonCamera cam;
  private boolean hasTargets;
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator photonEstimator;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private int tagID = -1;
  private Swerve swerve;
  private PhotonPipelineResult result;
  private double lastEstTimestamp = 0;
  private Transform3d robotToCam;

  public PhotonCam (String camName, Transform3d RobotToCam, Swerve swerve, AprilTagFieldLayout fieldLayout) {
    this.camName = camName;
    this.fieldLayout = fieldLayout;
    cam = new PhotonCamera(camName);
    this.swerve = swerve;
    hasTargets = false;
    this.robotToCam = RobotToCam;

    photonEstimator = new PhotonPoseEstimator(
      fieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      cam,
      robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void update(){
    result = cam.getLatestResult();
    var visionEst = getEstimatedGlobalPose();
    SmartDashboard.putBoolean("Estimator is Present", visionEst.isPresent());
    visionEst.ifPresent(
      est -> {
        // Calculate robot's field relative pose
        var tar = result.getBestTarget();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
          tar.getBestCameraToTarget(), 
          fieldLayout.getTagPose(tar.getFiducialId()).get(), 
          robotToCam);
          
        var estPose = est.estimatedPose.toPose2d();
        var estStdDevs = getEstimatedStdDevs(estPose);
        SmartDashboard.putString("Vision Estimated Pose", estPose.toString());
        SmartDashboard.putString("Test Pose", robotPose.toPose2d().toString());

        filterAndAddVisionPose(est);
      }
    );
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    var visionEstimation = photonEstimator.update(cam.getLatestResult());
    double latesttimeStamp = cam.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latesttimeStamp - lastEstTimestamp) > 1e-5;
    if (newResult){lastEstTimestamp = latesttimeStamp;}
    return visionEstimation;
  }

  public Matrix<N3, N1> getEstimatedStdDevs(Pose2d estimatedPose){
    var estStdDevs = VecBuilder.fill(2.0, 2.9, 2.9);
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

  public void filterAndAddVisionPose(EstimatedRobotPose pose){
    Matrix<N3, N1> cov = VecBuilder.fill(0.4, 0.4, 0.4);
    /* 
    double distance = 0;
    for (var t : pose.targetsUsed){
      distance += t.getBestCameraToTarget().getTranslation().getNorm() / pose.targetsUsed.size();
    }
    if (pose.targetsUsed.size() > 1){
      // has multiple tags
      double distance2 = Math.max(Math.pow(distance * 0.4, 2), 0.7);
      cov = VecBuilder.fill(distance2, distance2, 100);
    }
    if (!DriverStation.isDisabled()){
      if (pose.targetsUsed.size() == 1){
        if (Math.abs(pose.estimatedPose.getZ()) > 1.0 
          || pose.estimatedPose
            .minus(new Pose3d(swerve.getPose()))
            .getTranslation()
            .getNorm()
          > 1.0
          || distance > 7.0) {
        return;}
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


}
