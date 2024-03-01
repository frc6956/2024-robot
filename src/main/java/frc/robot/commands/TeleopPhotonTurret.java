// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class TeleopPhotonTurret extends Command {
  /** Creates a new TeleopPhotonTurret. */
  Swerve swerve;
  PhotonVision photonVision;
  Wrist wrist;

  double targetWrist;

  DoubleSupplier translationSup;
  DoubleSupplier strafeSup;
  DoubleSupplier rotationSup;

  private SlewRateLimiter m_xAxisLimiter;
  private SlewRateLimiter m_yAxisLimiter;

  public TeleopPhotonTurret(DoubleSupplier translationSup, DoubleSupplier strafeSup,Swerve swerve, PhotonVision photonVision) {
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xAxis = MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.stickDeadband);
    double yAxis = MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.stickDeadband);

    double xAxisSquared = xAxis * xAxis * Math.signum(xAxis);
    double yAxisSquared = yAxis * yAxis * Math.signum(yAxis);

    double xAxisFiltered = m_xAxisLimiter.calculate(xAxisSquared);
    double yAxisFiltered = m_yAxisLimiter.calculate(yAxisSquared);
    if (photonVision.hasAprilTag()){

    PhotonPipelineResult result = photonVision.getLatestResult();
      if (photonVision.hasCenterSpeaker(result) && result.hasTargets() && result != null){
        PhotonTrackedTarget target = photonVision.getSpeakerCenterTarget(result);
        System.out.println("has target");
        PIDController rotController = new PIDController(VisionConstants.visionP, VisionConstants.visionI, VisionConstants.visionD);
        
        rotController.enableContinuousInput(-180, 180);

        double rotate = rotController.calculate(
          swerve.getYaw(),
          swerve.getYaw() + target.getYaw()
        );

        swerve.drive(
            new Translation2d(-xAxisFiltered, -yAxisFiltered).times(DriveConstants.MaxSpeed), 
            rotate, true, true, false, false);
      } else {
        swerve.drive(
            new Translation2d(-xAxisFiltered, -yAxisFiltered).times(DriveConstants.MaxSpeed), 
            0, true, true, false, false);
      }
    } else {
      swerve.drive(
            new Translation2d(-xAxisFiltered, -yAxisFiltered).times(DriveConstants.MaxSpeed), 
            0, true, true, false, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
