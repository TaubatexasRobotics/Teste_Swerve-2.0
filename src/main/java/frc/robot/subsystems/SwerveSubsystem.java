package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;

public class SwerveSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("RobotPhysics");
    NetworkTableEntry HeadingEntry = table.getEntry("Heading");

    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    SwerveModuleState[] states = new SwerveModuleState[]{
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    DoublePublisher headingPublisher;
    AutoBuilder autoBuilder;
    NTSendableBuilder builder;
    RobotConfig config;
    NetworkTable m_table;
    boolean doRejectUpdate;
    
    public final static Pigeon2 gyro = new Pigeon2(62);

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), getSwerveModulePosition(), new Pose2d());
    
    public SwerveSubsystem() {
        
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPoseEstimator, // Robot pose supplier
            this::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisRelativeSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(50.5, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
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

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

    }

    public void zeroHeading() {
        gyro.reset();
    }

    public void setHeading(double angle) {
        gyro.setYaw(angle);
    }

    public double getAngle() {
        return gyro.getYaw().getValueAsDouble();
    }
    
    public double getHeading() {
        return Math.IEEEremainder(getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPoseEstimator() {
        Pose2d pose = poseEstimator.getEstimatedPosition();
        return pose;
    }

    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getSwerveModulePosition(), pose);
    }   

    public double getForwardPosition(){
        return frontLeft.getDrivePosition();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetAllEncoders() {
        backRight.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        frontLeft.resetEncoders();
    }

    public void resetSwerve(){
        zeroHeading();
        resetAllEncoders();
        resetPoseEstimator(new Pose2d());
    }

    public ChassisSpeeds getChassisSpeed() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleState());
    }

    public ChassisSpeeds getChassisRelativeSpeed() {
        ChassisSpeeds Speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleState());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            Speeds.vxMetersPerSecond, Speeds.vyMetersPerSecond, Speeds.omegaRadiansPerSecond, getRotation2d());
    }

    public void setModuleStatesFromChassisSpeeds(ChassisSpeeds chassisSpeeds) { 
        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            
        frontLeft.setDesiredState(desiredStates[0], frontLeft.getModuleRotation2d(), false);
        frontRight.setDesiredState(desiredStates[1], frontRight.getModuleRotation2d(), false);
        backLeft.setDesiredState(desiredStates[2], backLeft.getModuleRotation2d(), false);
        backRight.setDesiredState(desiredStates[3], backRight.getModuleRotation2d(), false); 
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) { 
        frontLeft.setDesiredState(desiredStates[0], frontLeft.getModuleRotation2d(), true);
        frontRight.setDesiredState(desiredStates[1], frontRight.getModuleRotation2d(), false);
        backLeft.setDesiredState(desiredStates[2], backLeft.getModuleRotation2d(), false);
        backRight.setDesiredState(desiredStates[3], backRight.getModuleRotation2d(), false); 
    }
    
    public SwerveModuleState[] getSwerveModuleState() {
        final SwerveModuleState[] PosSwerveModuleStates = new SwerveModuleState[4];
        PosSwerveModuleStates[0] = frontLeft.getState();
        PosSwerveModuleStates[1] = frontRight.getState();
        PosSwerveModuleStates[2] = backLeft.getState();
        PosSwerveModuleStates[3] = backRight.getState();
        
        return PosSwerveModuleStates;
    }

    public SwerveModulePosition[] getSwerveModulePosition() {
        final SwerveModulePosition[] PosSwerveModulePositions = new SwerveModulePosition[4];
        PosSwerveModulePositions[0] = frontLeft.getPosition();
        PosSwerveModulePositions[1] = frontRight.getPosition();
        PosSwerveModulePositions[2] = backLeft.getPosition();
        PosSwerveModulePositions[3] = backRight.getPosition();
        
        return PosSwerveModulePositions;
    }
    
    public void setRobotSpeed(double xSpeed, double ySpeed, double angularSpeed) {
        ChassisSpeeds chassisSpeeds;
        
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, angularSpeed, getRotation2d());
            
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            
            setModuleStates(moduleStates);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        
        setModuleStates(moduleStates);
    }
        
    public void setRobotOrientedSpeed(double xSpeed, double ySpeed, double angularSpeed) {
        ChassisSpeeds chassisSpeeds;
            
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angularSpeed);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        setModuleStates(moduleStates);
    }
    
    public double[] getPoseArray(Pose2d pose){
        double x = pose.getX();
        double y = pose.getY();
        double omega = pose.getRotation().getDegrees();

        double[] array = {x,y,omega};
        return array;
    }

    public String[] getSwerveStateArray(){
        SwerveModuleState[] state = getSwerveModuleState();

        String[] array = {state[0].toString(),state[1].toString(),state[2].toString(),state[3].toString()};

        return array;
    }

    public void setModuleSpeed(){
        //frontLeft.setDriveSpeed(0.05);
        //frontLeft.setTurningSpeed(0.05);

        //backLeft.setDriveSpeed(0.05);
        //backLeft.setTurningSpeed(0.05);

        //backRight.setDriveSpeed(0.05);
        //backRight.setTurningSpeed(0.05);

        frontRight.setDriveSpeed(0.05);
        frontRight.setTurningSpeed(0.05);
    }

    // ==========================================================

    StructArrayPublisher<SwerveModuleState> publisherStates = NetworkTableInstance.getDefault().getTable("RobotPhysics")
    .getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();

    StructPublisher<Pose2d> publisherPose = NetworkTableInstance.getDefault().getTable("RobotPhysics")
    .getStructTopic("PoseEstimator", Pose2d.struct).publish();

    @Override
    public void periodic() {
        publisherStates.set(getSwerveModuleState());
        publisherPose.set(getPoseEstimator());
        HeadingEntry.setDouble(Math.toRadians(getHeading()));

        poseEstimator.update(getRotation2d(), getSwerveModulePosition());
    }
}