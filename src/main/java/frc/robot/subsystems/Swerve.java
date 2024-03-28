package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Translation2dPlus;
import frc.robot.Constants.*;
import frc.robot.Constants.ModuleConstants.*;
import java.util.Arrays;

/**
 * The Swerve subsystem controls the swerve drive system of the robot. It handles the odometry, pose
 * estimation, module control, and field-centric driving.
 */
public class Swerve extends SubsystemBase {
  private final SwerveDriveOdometry swerveOdometry;
  private final SwerveModule[] mSwerveMods;
  private final Pigeon2 gyro;
  final Field2d m_field = new Field2d();
  public Pose3d visionPose3d;

  /** The positions of the swerve drive wheels on the robot. */
  private static final Translation2d[] WHEEL_POSITIONS =
      Arrays.copyOf(DriveConstants.moduleTranslations, DriveConstants.moduleTranslations.length);

  /** Represents a network table used for communication between the robot and the driver station. */
  NetworkTable SwerveTable;

  /** The pose estimator for the Swerve drive subsystem. */
  public SwerveDrivePoseEstimator poseEstimator;

  /** The offset value for the gyro sensor. */
  private double gyroOffset;

  /**
   * Represents a swerve drive subsystem. This class initializes and configures the swerve modules,
   * gyro, odometry, pose estimator, and path follower. It provides methods for controlling the
   * swerve drive system and accessing its state.
   */
  public Swerve() {
    gyro = new Pigeon2(DriveConstants.GyroID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(
              ModFL.moduleNumber,
              ModFL.name,
              ModFL.driveMotorID,
              ModFL.angleMotorID,
              ModFL.encoderID,
              ModFL.angleOffset,
              false),
          new SwerveModule(
              ModFR.moduleNumber,
              ModFR.name,
              ModFR.driveMotorID,
              ModFR.angleMotorID,
              ModFR.encoderID,
              ModFR.angleOffset,
              false),
          new SwerveModule(
              ModBL.moduleNumber,
              ModBL.name,
              ModBL.driveMotorID,
              ModBL.angleMotorID,
              ModBL.encoderID,
              ModBL.angleOffset,
              false),
          new SwerveModule(
              ModBR.moduleNumber,
              ModBR.name,
              ModBR.driveMotorID,
              ModBR.angleMotorID,
              ModBR.encoderID,
              ModBR.angleOffset,
              false),
        };

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    visionPose3d = new Pose3d();

    swerveOdometry =
        new SwerveDriveOdometry(
            DriveConstants.swerveKinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0, 0.0)));

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.swerveKinematics,
            getHeading(),
            getModulePositions(),
            swerveOdometry.getPoseMeters() // ,
            // stateStdDevs,
            // visionStdDevs
            );

    AutoBuilder.configureHolonomic(
        this::getOdomPose,
        this::resetPose,
        () -> DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates()),
        speeds -> {
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          SwerveModuleState[] swerveModuleStates =
              DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
          setModuleStates(swerveModuleStates);
        },
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your
            // Constants class
            new PIDConstants( // Translation PID constants
                Auto.AutoDriveP, Auto.AutoDriveI, Auto.AutoDriveD),
            new PIDConstants( // Rotation PID constants
                Auto.AutoTurnP, Auto.AutoTurnI, Auto.AutoTurnD),
            Auto.MaxSpeed, // Max module speed, in m/s
            DriveConstants
                .CenterToWheel, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig(
                true, true) // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // SwerveTable =
    // NetworkTableInstance.getDefault().getTable(VisionConstants.camName);
  } // end of Swerve Contructor

  /**
   * This method is called periodically to update the state of the Swerve subsystem. It updates the
   * swerve odometry, pose estimator, robot pose on the field, and displays relevant data on the
   * SmartDashboard.
   */
  @Override
  public void periodic() {

    swerveOdometry.update(getHeading(), getModulePositions());

    poseEstimator.update(getHeading(), getModulePositions());

    m_field.setRobotPose(poseEstimator.getEstimatedPosition());

    SmartDashboard.putData(m_field);

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(mod.name + " Encoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(mod.name + " Integrated", mod.getPosition().angle.getDegrees());
    }

    SmartDashboard.putString(
        "Estimated Robot Pose", poseEstimator.getEstimatedPosition().toString());
  }

  /**
   * Returns the Field2d object associated with this Swerve subsystem.
   *
   * @return the Field2d object
   */
  public Field2d getField2d() {
    return m_field;
  }

  /**
   * Drives the swerve robot based on the given translation and rotation values.
   *
   * @param translation The translation vector representing the robot's desired movement in the
   *     field coordinate system.
   * @param rotation The robot's desired rotation in radians per second.
   * @param fieldRelative Specifies whether the translation and rotation values are relative to the
   *     field coordinate system.
   * @param isOpenLoop Specifies whether the robot should be driven in open loop control mode.
   * @param isEvading Specifies whether the robot is currently evading an obstacle.
   * @param isLocked Specifies whether the swerve modules should be locked in place.
   */
  public void drive(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      boolean isEvading,
      boolean isLocked) {
    if (isLocked) {

      final SwerveModuleState[] swerveModuleStates =
          new SwerveModuleState[] {
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
          };

      for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      }

    } else {

      final Translation2d centerOfRotation;

      if (isEvading && fieldRelative) {
        centerOfRotation = getCenterOfRotation(translation.getAngle(), rotation);
      } else {
        centerOfRotation = new Translation2d();
      }

      final ChassisSpeeds chassisSpeeds;

      if (fieldRelative) {

        chassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getHeading());

      } else {

        chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
      }

      final var swerveModuleStates =
          DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MaxSpeed);

      for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      }
    }
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param pose The pose measurement from vision.
   * @param timestamp The timestamp of the measurement.
   * @param cov The covariance matrix of the measurement.
   */
  public void addVisionMeasurement(Pose3d pose, double timestamp, Matrix<N3, N1> cov) {
    visionPose3d = pose;
    poseEstimator.addVisionMeasurement(pose.toPose2d(), timestamp, cov);
  }

  /**
   * Returns the yaw angle of the robot.
   *
   * @return The yaw angle in degrees.
   */
  public double getYaw() {
    return (DriveConstants.GyroInvert)
        ? 180 - (gyro.getYaw().getValueAsDouble() - gyroOffset)
        : gyro.getYaw().getValueAsDouble() - gyroOffset;
  }

  /**
   * Returns the heading of the robot as a Rotation2d object.
   *
   * @return the heading of the robot
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getYaw());
  }

  /**
   * Returns the pitch angle of the robot.
   *
   * @return the pitch angle of the robot
   */
  public double getPitch() {
    return gyro.getPitch().getValueAsDouble();
  }

  /**
   * Returns the roll angle of the robot.
   *
   * @return the roll angle of the robot
   */
  public double getRoll() {
    return gyro.getRoll().getValueAsDouble();
  }

  /**
   * Returns the current pose of the robot.
   *
   * @return the current pose of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the current pose of the swerve subsystem in the field coordinate system.
   *
   * @return the current pose of the swerve subsystem
   */
  public Pose2d getOdomPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Retrieves the current state of all swerve modules.
   *
   * @return an array of SwerveModuleState objects representing the state of each swerve module
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * Returns an array of the desired SwerveModuleState for each swerve module.
   *
   * @return an array of SwerveModuleState representing the desired state for each swerve module
   */
  public SwerveModuleState[] getDesiredSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getDesiredState();
    }
    return states;
  }

  /**
   * Retrieves the positions of all swerve modules.
   *
   * @return An array of SwerveModulePosition objects representing the positions of each swerve
   *     module.
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /**
   * Represents a 2D translation in the Cartesian coordinate system. The Translation2d class is used
   * to store and manipulate the x and y coordinates of a point in 2D space.
   */
  private Translation2d getCenterOfRotation(final Rotation2d direction, final double rotation) {
    final var here = new Translation2dPlus(1.0, direction.minus(getHeading()));

    var cwCenter = WHEEL_POSITIONS[0];
    var ccwCenter = WHEEL_POSITIONS[WHEEL_POSITIONS.length - 1];

    for (int i = 0; i < WHEEL_POSITIONS.length - 1; i++) {
      final var cw = WHEEL_POSITIONS[i];
      final var ccw = WHEEL_POSITIONS[i + 1];

      if (here.isWithinAngle(cw, ccw)) {
        cwCenter = ccw;
        ccwCenter = cw;
      }
    }

    // if clockwise
    if (Math.signum(rotation) == 1.0) {
      return cwCenter;
    } else if (Math.signum(rotation) == -1.0) {
      return ccwCenter;
    } else {
      return new Translation2d();
    }
  }

  /** Resets the gyro offset to the current yaw value. */
  public void zeroGyro() {
    gyroOffset = gyro.getYaw().getValueAsDouble();
  }

  /**
   * Sets the desired states for each swerve module. The desired states are provided as an array of
   * SwerveModuleState objects. The wheel speeds are desaturated to ensure they do not exceed the
   * maximum speed defined in DriveConstants. Each swerve module's desired state is set using the
   * corresponding element from the desiredStates array.
   *
   * @param desiredStates An array of SwerveModuleState objects representing the desired states for
   *     each swerve module.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MaxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], true);
    }
  }

  /** Resets all swerve modules to their absolute position. */
  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Resets the field orientation of the swerve subsystem. This method sets the field orientation to
   * the default value (zero rotation).
   */
  public void resetFieldOrientation() {
    Rotation2d redOrBlueZero = new Rotation2d();
    resetPose(new Pose2d(getPose().getTranslation(), redOrBlueZero));
  }

  /**
   * Resets the pose of the swerve subsystem to the specified pose. This method updates the swerve
   * odometry and pose estimator with the new pose.
   *
   * @param pose The new pose to set for the swerve subsystem.
   */
  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  // Rotates by a passed amount of degrees
  /**
   * Rotates the robot by the specified target angle in degrees using a PID controller.
   *
   * @param target The target angle in degrees to rotate the robot.
   */
  public void rotateDegrees(double target) {
    try (PIDController rotController =
        new PIDController(DriveConstants.TurnP, DriveConstants.TurnI, DriveConstants.TurnD)) {
      rotController.enableContinuousInput(DriveConstants.MinAngle, DriveConstants.MaxAngle);

      double rotate = rotController.calculate(getYaw(), getYaw() + target);

      drive(new Translation2d(0, 0), rotate, false, true, false, false);
    }
  }
}
