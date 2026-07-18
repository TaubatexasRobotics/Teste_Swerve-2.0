package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveMotorConfig;
    private final SparkMax turningMotor;
    private final SparkMaxConfig turningMotorConfig;
    private double turningMotorSpeed;
    private final CANcoder absoluteEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new CANcoder(absoluteEncoderId);
        driveMotor = new TalonFX(driveMotorId);
        
        driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.Feedback.withSensorToMechanismRatio(0.9);
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveMotorConfig);

        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        turningMotorConfig = new SparkMaxConfig();

        turningMotorConfig.inverted(turningMotorReversed);
        turningMotorConfig.idleMode(IdleMode.kCoast);
                    
        turningMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        
        // turningMotorConfig.closedLoop.p(ModuleConstants.kPTurning).i(0).d(0);

        // turningMotorConfig.closedLoop.maxMotion
        // .allowedClosedLoopError(1)
        // .maxAcceleration(10000)
        // .maxVelocity(10000);

        //turningMotorConfig.closedLoop
        //.positionWrappingEnabled(true)
        //.positionWrappingInputRange(0, 2 * Math.PI);

        turningMotor.configure(turningMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        turningPidController = new PIDController(0.5, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setTolerance(0.0);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getDriveCurrent() {
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getTurningPosition() {
        return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue().in(Radians);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public Rotation2d getModuleRotation2d() {
        Rotation2d angleRotation2d = new Rotation2d(getAbsoluteEncoderRad());
        return angleRotation2d;
    }

    public void resetEncoders() {
        //driveMotor.setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getModuleHeading(){
        return getAbsoluteEncoderRad();
    }

    public void setDesiredState(SwerveModuleState state, Rotation2d rotation2d, Boolean plotar) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        state.optimize(rotation2d);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        turningMotorSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(turningMotorSpeed);

        if (plotar){
            SmartDashboard.putNumber("POSITION", getTurningPosition());
            SmartDashboard.putNumber("SETPOINT", state.angle.getRadians());
            SmartDashboard.putNumber("SPEED", turningMotorSpeed);
        }
        
        //turningMotor.getClosedLoopController().setReference(state.angle.getRadians(), ControlType.kMAXMotionPositionControl);
    }

    public void setDesiredStateNoOptimized(SwerveModuleState state, Rotation2d rotation2d, Boolean plotar) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotorSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        
        if (plotar){
            SmartDashboard.putNumber("POSITION", getTurningPosition());
            SmartDashboard.putNumber("SETPOINT", state.angle.getRadians());
            SmartDashboard.putNumber("SPEED", turningMotorSpeed);
        }

        //turningMotor.getClosedLoopController().setReference(state.angle.getRadians(), ControlType.kMAXMotionPositionControl);
        turningMotor.set(turningMotorSpeed);
    }

    public void setTurningSpeed(double speed){
        turningMotor.set(speed);
    }

    public void setDriveSpeed(double speed){
        driveMotor.set(speed);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
