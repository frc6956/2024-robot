// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

public class TeleopPhotonTurret extends Command {
  /** Creates a new AimToSpeaker. */
  private Swerve swerve;
  private PhotonVision photonVision;
  private boolean isBlue;
  private boolean isDone = false;
  private PIDController rotController = new PIDController(VisionConstants.visionP, VisionConstants.visionI, VisionConstants.visionD);

  DoubleSupplier translationSup;
  DoubleSupplier strafeSup;
  DoubleSupplier rotationSup;

  private SlewRateLimiter m_xAxisLimiter;
  private SlewRateLimiter m_yAxisLimiter;


  public TeleopPhotonTurret(DoubleSupplier translationSup, DoubleSupplier strafeSup, Swerve swerve, PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.photonVision = photonVision;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    addRequirements(swerve);

    m_xAxisLimiter = new SlewRateLimiter(OperatorConstants.MagnitudeSlewRate);
    m_yAxisLimiter = new SlewRateLimiter(OperatorConstants.MagnitudeSlewRate);
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

    double xAxis = MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.stickDeadband);
    double yAxis = MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.stickDeadband);

    double xAxisSquared = xAxis * xAxis * Math.signum(xAxis);
    double yAxisSquared = yAxis * yAxis * Math.signum(yAxis);

    double xAxisFiltered = m_xAxisLimiter.calculate(xAxisSquared);
    double yAxisFiltered = m_yAxisLimiter.calculate(yAxisSquared);

    Pose2d targetPose;
    if (isBlue){
      targetPose = photonVision.getTagPose(7).get().toPose2d();
    } else {
      targetPose = photonVision.getTagPose(4).get().toPose2d();
    }

    double yawError = PhotonUtils.getYawToPose(swerve.getPose(), targetPose).getDegrees();
    
    //double rotate = rotController.calculate(swerve.getPose().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
    double rotate = rotController.calculate(
      swerve.getYaw(),
      swerve.getYaw() + yawError
    );
    System.out.println(rotate);
    swerve.drive(
            new Translation2d(-xAxisFiltered, -yAxisFiltered).times(DriveConstants.MaxSpeed), 
            rotate, true, true, false, false);

    SmartDashboard.putNumber("Yaw Error", yawError);
    SmartDashboard.putNumber("TargetPoseRotation",  targetPose.getRotation().getDegrees());
    SmartDashboard.putNumber("BotPoseRotation", swerve.getPose().getRotation().getDegrees());
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
