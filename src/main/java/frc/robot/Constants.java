package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static class ModuleConstants{

        /* Front Left Module */
        public static final class ModFL {
            public static final String name = "Front Left Module";
            public static final int moduleNumber = 0;
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-13.7);
        }

        /* Front Right Module */
        public static final class ModFR {
            public static final String name = "Front Right Module";
            public static final int moduleNumber = 1;
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(146.1);
        }
        
        /* Back Left Module */
        public static final class ModBL {
            public static final String name = "Back Left Module";
            public static final int moduleNumber = 2;
            public static final int driveMotorID = 17;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(134.03);
        }

        /* Back Right Module */
        public static final class ModBR { 
            public static final String name = "Back Right Module";
            public static final int moduleNumber = 3;
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(122.9);
        }
    }



    public static class DriveConstants{

        public static final double MinAngle = -180;
        public static final double MaxAngle = 180;

        public static final int GyroID = 0;

        public static final double TrackWidth = Units.inchesToMeters(26);
        public static final double WheelBase = Units.inchesToMeters(26);
        public static final double WheelDiameter = Units.inchesToMeters(4);
        public static final double WheelCircumference = WheelDiameter * Math.PI;

        public static final double CenterToWheel =
                Math.sqrt(Math.pow(WheelBase / 2.0, 2) + Math.pow(TrackWidth / 2.0, 2));

        public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(WheelBase / 2.0, TrackWidth / 2.0),
            new Translation2d(WheelBase / 2.0, -TrackWidth / 2.0),
            new Translation2d(-WheelBase / 2.0, TrackWidth / 2.0),
            new Translation2d(-WheelBase / 2.0, -TrackWidth / 2.0)
        };

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(moduleTranslations);

        /* Speed */
        public static final double MaxSpeed = 0;
        /* Feed Forward */
        public static final double DriveKS = 0;
        public static final double DriveKV = 0;
        public static final double DriveKA = 0;


        /* Drive PID */
        public static final double DriveP = 0.0;
        public static final double DriveI = 0.0;
        public static final double DriveD = 0.0;

        public static final double DriveF = 0.0;

        /* Turn PID */
        public static final double TurnP = 0.0;
        public static final double TurnI = 0.0;
        public static final double TurnD = 0.0;

        public static final double TurnF = 0.0;

        /* Drive Motor */
        public static final int DriveContinuousCurrentLimit = 0;
        public static final boolean DriveInvert = false;
        public static final IdleMode DriveIdleMode = IdleMode.kBrake;

        /* Angle Motor */
        public static final int AngleContinuousCurrentLimit = 0;
        public static final boolean AngleInvert = false;
        public static final IdleMode AngleIdleMode = IdleMode.kCoast;
        
        /* Conversion Factors */
        public static final double DriveConversionFactor = 0.0;
        public static final double AngleConversionFactor = 0.0;

        public static final double DriveVelocityConversionFactor = 0.0;

        /* Voltage Compensation */
        public static final int DriveVoltageComp = 0;
        public static final int AngleVoltageComp = 0;

    }


    public static class Auto{

        /* PID */
        public static final double AutoTurnP = 0.0;
        public static final double AutoTurnI = 0.0;
        public static final double AutoTurnD = 0.0;

        public static final double AutoDriveP = 0.0;
        public static final double AutoDriveI = 0.0;
        public static final double AutoDriveD = 0.0;

    }
}
