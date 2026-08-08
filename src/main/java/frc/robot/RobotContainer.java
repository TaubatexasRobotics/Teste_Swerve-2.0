package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class RobotContainer {
    
    // ======================== INSTANCIA OS SUBSISTEMAS =========================
    private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    // ============================================================================
    
    // ======================== INSTANCIA OS JOYSTICKS =========================
    public final static Joystick Joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    // ============================================================================
    
    // ======================== INSTANCIA O SHUFFLEBOARD =========================
    ShuffleboardTab intakeTab, limelightTab, climbTab, elevatorTab;
    public GenericEntry TXLSetpointEntry, TZLSetpointEntry, RYLSetpointEntry, TXRSetpointEntry, TZRSetpointEntry , RYRSetpointEntry;
    public GenericEntry TXLSpeedEntry, TZLSpeedEntry, RYLSpeedEntry, TXRSpeedEntry, TZRSpeedEntry, RYRSpeedEntry;

    // ShuffleBoardConstants shuffleBoardConstants = new ShuffleBoardConstants();
    // ============================================================================

    public RobotContainer() {
        
        // =========================== COMANDOS PADRÕES ===========================
        // Comando padrão swerve operado por joystick
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem, () -> Joystick1.getRawAxis(1), () -> -Joystick1.getRawAxis(0),
                () -> -Joystick1.getRawAxis(4), () -> Joystick1.getRawButton(OIConstants.kResetEncodersButtonIdx)));

        // Atribui as funções para cada botão do Controle
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //new JoystickButton(Joystick1, 1).onTrue(new InstantCommand(() -> climbSubsystem.changeClimbSetpoint(climbConstants.robotUpPosition)));

    };


    // ========== EXECUTA QUANDO O ROBÔ INICIAR ==========
    public void doWhenRobotInit() {
    }
    // =======================================================

    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenAutoInit() {
    }
    // =======================================================
    
    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenTeleopInit() {

    } 
    // =======================================================

    // Executa a opção escolhida de Autônomo
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
    // ========================================================

}
