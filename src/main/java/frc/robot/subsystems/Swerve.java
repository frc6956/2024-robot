package frc.robot.subsystems;

import java.util.Arrays;

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

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;
    final Field2d m_field = new Field2d();
    public Pose3d visionPose3d;


    private static final Translation2d[] WHEEL_POSITIONS =
        Arrays.copyOf(DriveConstants.moduleTranslations, DriveConstants.moduleTranslations.length);

    NetworkTable SwerveTable;

    public SwerveDrivePoseEstimator poseEstimator;
    private double gyroOffset;

    public Swerve() {
        gyro = new Pigeon2(DriveConstants.GyroID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
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

        swerveOdometry = new SwerveDriveOdometry(
            DriveConstants.swerveKinematics,
            getHeading(),
            getModulePositions()
        );

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.swerveKinematics, 
            getHeading(), 
            getModulePositions(), 
            swerveOdometry.getPoseMeters());

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            () -> DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates()),
            speeds -> {
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                SwerveModuleState[] swerveModuleStates =
                    DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
                setModuleStates(swerveModuleStates);
            },
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                // Constants class
                new PIDConstants( // Translation PID constants
                    Auto.AutoDriveP,
                    Auto.AutoDriveI,
                    Auto.AutoDriveD
                ),
                new PIDConstants( // Rotation PID constants
                    Auto.AutoTurnP,
                    Auto.AutoTurnI,
                    Auto.AutoTurnD
                ),
                Auto.MaxSpeed, // Max module speed, in m/s
                DriveConstants.CenterToWheel, // Drive base radius in meters. Distance from robot center to
                                                  // furthest module.
                new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
            ), 
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);

       //SwerveTable = NetworkTableInstance.getDefault().getTable(VisionConstants.camName);
    } // end of Swerve Contructor

    @Override
    public void periodic() {

        swerveOdometry.update(
            getHeading(), 
            getModulePositions());

        poseEstimator.updateWithTime(
            Timer.getFPGATimestamp(), 
            getHeading(), 
            getModulePositions());

        m_field.setRobotPose(poseEstimator.getEstimatedPosition());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(mod.name + " Encoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(mod.name + " Integrated", mod.getPosition().angle.getDegrees());
        }
    
        SmartDashboard.putString
        ("Estimated Robot Pose", poseEstimator.getEstimatedPosition().toString());
    }


    public Field2d getField2d() {
        return m_field;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isEvading, boolean isLocked) {
        if(isLocked) {

            final SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
            };

            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }

        } else {

            final Translation2d centerOfRotation;

            if(isEvading && fieldRelative) {
                centerOfRotation = getCenterOfRotation(translation.getAngle(), rotation);
            } else {
                centerOfRotation = new Translation2d();
            }

            final ChassisSpeeds chassisSpeeds;
        
            if(fieldRelative) {

                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getHeading()
                    );

            } else {

                chassisSpeeds = new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation);
            } 

            final var swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MaxSpeed);

            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }

        }
    }

    public void addVisionMeasurement(Pose3d pose, double timestamp, Matrix<N3, N1> cov){
        visionPose3d = pose;
        poseEstimator.addVisionMeasurement(pose.toPose2d(), timestamp);
    }

    public double getYaw() {
        return (DriveConstants.GyroInvert) ?
                180 - (gyro.getYaw().getValueAsDouble() - gyroOffset) :
                gyro.getYaw().getValueAsDouble() - gyroOffset;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getDesiredSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getDesiredState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

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

    public void zeroGyro() {
        gyroOffset = gyro.getYaw().getValueAsDouble();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MaxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void resetFieldOrientation(){
        Rotation2d redOrBlueZero = new Rotation2d();
        resetPose(new Pose2d(getPose().getTranslation(), redOrBlueZero));
    }

    public void resetPose(Pose2d pose){
        swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    private void resetPoseToVision(){
        resetPose(visionPose3d.toPose2d());
    }

    // Rotates by a passed amount of degrees
    public void rotateDegrees(double target) {
        try (
            PIDController rotController = new PIDController(
                DriveConstants.TurnP,
                DriveConstants.TurnI,
                DriveConstants.TurnD
            )
        ) {
            rotController.enableContinuousInput(DriveConstants.MinAngle, DriveConstants.MaxAngle);

            double rotate = rotController.calculate(getYaw(), getYaw() + target);

            drive(new Translation2d(0, 0), rotate, false, true, false, false);
        }
    }
}