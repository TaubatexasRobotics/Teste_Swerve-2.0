package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class Constants {
    public static final class ModuleConstants{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
        public static final double kDriveMotorGearRatio = 1.0 / 6.75;
        public static final double kTurningMotorGearRatio = 1.0 / 26;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.3;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.56;
        // Distance between right and left wheels
        public static final double kWheelBase = 0.56;
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
                // Front Left 
                // Front Right
                // Back Left 
                // Back Right

        public static final int kFrontLeftDriveMotorPort = 46;
        public static final int kFrontRightDriveMotorPort = 47;
        public static final int kBackLeftDriveMotorPort = 48;
        public static final int kBackRightDriveMotorPort = 49;

        public static final int kFrontLeftTurningMotorPort = 56;
        public static final int kFrontRightTurningMotorPort = 57;
        public static final int kBackLeftTurningMotorPort = 58;
        public static final int kBackRightTurningMotorPort = 59;

        public static final int kElevatorMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 36;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 37;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 38;
        public static final int kBackRightDriveAbsoluteEncoderPort = 39;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kNoteCollectMaxSpeed = 2.5;

        public static double kSpeed = 1;
        public static double kSpeedAngular = 1;
        public static boolean lowSpeed = true;

        public static double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static double kTeleDriveMaxAngularSpeedRadiansPerSecond = // 
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double ksVolts = 0.29565;
        public static final double kvVoltSecondsPerMeter = 0.8224;
        public static final double kaVoltSecondsSquaredPerMeter = 1.4512;
        public static final double kPDriveVel = 1.1949;

    }

    public static final class XboxConstants {
        public static final int kResetEncodersButtonIdx = 3;
        
    }

    public static final class LogitechConstants {
        public static final int kResetEncodersButtonIdx = 1;

    }

    public static final class OIConstants {
        public static Boolean isXbox = DriverStation.getJoystickIsXbox(0);

        public static double getGyroAxis(Joystick joystick){
            return isXbox ? joystick.getRawAxis(4) : joystick.getZ();
        }

        // ============================ BOTÕES DO CONTROLE ===========================
        //Controle Swerve
        public static final int kResetEncodersButtonIdx = isXbox ? XboxConstants.kResetEncodersButtonIdx : LogitechConstants.kResetEncodersButtonIdx;
    
        // ============================================================================

        public static final int kDriverControllerPort = 0;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 2;
        
        public static final double kDeadband = 0.05;  

    }

}

