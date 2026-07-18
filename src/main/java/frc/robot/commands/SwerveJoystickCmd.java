package frc.robot.commands;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.filter.SlewRateLimiter;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
    import java.util.function.Supplier;
    import frc.robot.Constants.DriveConstants;
    import frc.robot.Constants.OIConstants;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class SwerveJoystickCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> resetEncoderButton;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double xSpeed, ySpeed, turningSpeed, rate;
    private ChassisSpeeds chassisSpeeds;
    // ============================================================================

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction,
                Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
                Supplier<Boolean> resetEncoderButton) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.resetEncoderButton = resetEncoderButton;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        rate = 0.2;
    }

    @Override
    public void execute() {
        if (resetEncoderButton.get()){
            swerveSubsystem.resetSwerve();
        }
   
        // 1. Coleta a posição do Joystick
        xSpeed = xSpdFunction.get();
        ySpeed = ySpdFunction.get();
        turningSpeed = turningSpdFunction.get();

        // 2. Aplica uma tolerância
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        
        xSpeed = xLimiter.calculate(xSpeed) * rate;
        ySpeed = yLimiter.calculate(ySpeed) * rate;
        turningSpeed = turningLimiter.calculate(turningSpeed) * rate;

        // 5. Calcula a velocidade do robô, em relação a arena
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        // 6. Calcula a velocidade individual de cada módulo Swerve
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 7. Move os módulos para a posição e aceleração desejadas
        swerveSubsystem.setModuleStates(moduleStates);
        SmartDashboard.putNumber("X speed", xSpeed);
        SmartDashboard.putNumber("Y speed", ySpeed);

        SmartDashboard.putNumber("X controle", xSpdFunction.get());
        SmartDashboard.putNumber("Y controle", ySpdFunction.get());

        //swerveSubsystem.setModuleSpeed();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
