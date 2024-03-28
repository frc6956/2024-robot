package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    public int moduleNumber;
    public String name;
    private boolean driveInvert;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    // private ThriftyEncoder angleEncoder;
    private CANcoder angleEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private RelativeEncoder driveEncoder;

    private SparkPIDController driveController;
    private SparkPIDController angleController;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(DriveConstants.DriveKS,
            DriveConstants.DriveKV, DriveConstants.DriveKA);

    /**
     * Represents a swerve module on the robot.
     * A swerve module consists of a drive motor, an angle motor, and an angle
     * encoder.
     * It is responsible for controlling the movement and rotation of a specific
     * wheel on the robot.
     */
    public SwerveModule(int moduleNumber, String name, int driveMotorID, int angleMotorID, int encoderID,
            Rotation2d angleOffset, boolean driveInvert) {
        this.moduleNumber = moduleNumber;
        this.name = name;
        this.angleOffset = angleOffset;
        this.driveInvert = driveInvert;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(encoderID);
        // angleEncoder = new ThriftyEncoder(encoderID);
        // Timer.delay(0.5);
        configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = m_angleMotor.getEncoder();
        angleController = m_angleMotor.getPIDController();
        // Timer.delay(0.5);
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = m_driveMotor.getEncoder();
        driveController = m_driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    /**
     * Sets the desired state of the SwerveModule.
     * 
     * @param desiredState The desired state of the SwerveModule.
     * @param isOpenLoop   A boolean indicating whether the control mode is open
     *                     loop or not.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        m_desiredState = desiredState;
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speed of the swerve module.
     * 
     * @param desiredState The desired state of the swerve module.
     * @param isOpenLoop   A boolean indicating whether the control is open loop or
     *                     closed loop.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.MaxSpeed;
            m_driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    driveFeedForward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    /**
     * Sets the angle of the swerve module to the desired state.
     * If the speed is less than 2% of the maximum speed, the module will not rotate
     * to prevent jittering.
     * 
     * @param desiredState The desired state of the swerve module.
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.MaxSpeed * 0.1))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 2%. Prevents Jittering.

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    /**
     * Returns the angle of the swerve module.
     *
     * @return the angle of the swerve module as a Rotation2d object
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    /**
     * Returns the rotation in radians from the angle encoder.
     *
     * @return the rotation in radians
     */
    public Rotation2d getCanCoder() {
        return Rotation2d.fromRadians(angleEncoder.getAbsolutePosition().getValue() * 2 * Math.PI);
    }
    /*
     * 
     * public Rotation2d getThriftyEncoder(){
     * return Rotation2d.fromRadians(angleEncoder.getPosition());
     * }
     */

    /**
     * Resets the integrated angle encoder to the absolute position.
     * The absolute position is calculated by subtracting the angle offset from the
     * current position of the CANCoder.
     */
    public void resetToAbsolute() {
        // integratedAngleEncoder.setPosition(0);
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        // double absolutePosition = getThriftyEncoder().getDegrees() -
        // angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    /**
     * Configures the angle encoder for the swerve module.
     * The default range for the CANCoder is 0-360 degrees for getAbsolutePosition.
     * Sets the sensor direction to counter-clockwise positive.
     */
    private void configAngleEncoder() {
        /* The default for a canCoder is 0-360 degrees for getAbsolutePosition */

        CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        angleEncoder.getConfigurator().apply(swerveCANcoderConfig);

    }

    /**
     * Configures the angle motor for the swerve module.
     * This method sets various parameters and configurations for the angle motor.
     */
    private void configAngleMotor() {
        m_angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_angleMotor, Usage.kPositionOnly);
        m_angleMotor.setSmartCurrentLimit(DriveConstants.AngleContinuousCurrentLimit);
        m_angleMotor.setInverted(DriveConstants.AngleInvert);
        m_angleMotor.setIdleMode(DriveConstants.AngleIdleMode);
        integratedAngleEncoder.setPositionConversionFactor(DriveConstants.AngleConversionFactor);
        integratedAngleEncoder.setVelocityConversionFactor(DriveConstants.AngleVelocityConversionFactor);
        angleController.setP(DriveConstants.TurnP);
        angleController.setI(DriveConstants.TurnI);
        angleController.setD(DriveConstants.TurnD);
        angleController.setFF(DriveConstants.TurnF);
        m_angleMotor.enableVoltageCompensation(Constants.voltageComp);
        Timer.delay(0.5);
        resetToAbsolute();
    }

    /**
     * Configures the drive motor for the swerve module.
     * This method sets various parameters and configurations for the drive motor.
     */
    private void configDriveMotor() {
        m_driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_driveMotor, Usage.kAll);
        m_driveMotor.setSmartCurrentLimit(DriveConstants.DriveContinuousCurrentLimit);
        m_driveMotor.setInverted(driveInvert);
        m_driveMotor.setIdleMode(DriveConstants.DriveIdleMode);
        driveEncoder.setVelocityConversionFactor(DriveConstants.DriveVelocityConversionFactor);
        driveEncoder.setPositionConversionFactor(DriveConstants.DriveConversionFactor);
        driveController.setP(DriveConstants.DriveP);
        driveController.setI(DriveConstants.DriveI);
        driveController.setD(DriveConstants.DriveD);
        driveController.setFF(DriveConstants.DriveF);
        m_driveMotor.enableVoltageCompensation(Constants.voltageComp);
        driveEncoder.setPosition(0.0);
    }

    /**
     * Retrieves the current state of the swerve module.
     * 
     * @return The current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(),
                getAngle());
    }

    /**
     * Returns the desired state of the SwerveModule.
     *
     * @return The desired state of the SwerveModule.
     */
    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

    /**
     * Returns the position of the swerve module.
     *
     * @return the position of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                getAngle());
    }
}