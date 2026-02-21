package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.*;
import frc.robot.commands.Swerve.*;
import frc.robot.commands.FuelIntake.*;
import frc.robot.commands.Climber.*;
import frc.robot.commands.Routines.*;
import frc.robot.util.Constants.*;
//import frc.robot.util.Constants.FuelIntakeConstants;


@SuppressWarnings("unused")
public class RobotContainer {

    /* ======================= */
    /* ===== SUBSYSTEMS ====== */
    /* ======================= */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final FuelIntakeSubsystem m_fuel = new FuelIntakeSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    private final LimelightSubsystem m_LimeLight = new LimelightSubsystem();
    private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    /* ======================= */
    /* ===== CONTROLLERS ==== */
    /* ======================= */
    private final XboxController driverJoystick =
            new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController codriverJoystick =
            new XboxController(OIConstants.kCoDriverControllerPort);

    /* ======================= */
    /* ===== AUTONOMOUS ===== */
    /* ======================= */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private static final String kDefaultAuto = "BlueBLJustMove";

    /* ======================= */
    /* ====== PID ========= */
    /* ======================= */
    public final PIDController xController =
            new PIDController(AutoConstants.kPXController, 0, 0);
    public final PIDController yController =
            new PIDController(AutoConstants.kPYController, 0, 0);
    public final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    0,
                    AutoConstants.kIThetaController,
                    AutoConstants.kDThetaController,
                    AutoConstants.kThetaControllerConstraints
            );

    /* ======================= */
    /* ===== CONSTRUCTOR ==== */
    /* ======================= */
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        buildShuffleboard();
    }

    /* ======================= */
    /* == DEFAULT COMMANDS == */
    /* ======================= */
    private void configureDefaultCommands() {
        swerveSubsystem.setDefaultCommand(
                new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                        () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                        () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
                )
        );

        m_fuel.setDefaultCommand(
                new FuelIntakeJoystick(
                        m_fuel,
                        () -> codriverJoystick.getRawAxis(OIConstants.kCodriverRTrigger),
                        () -> codriverJoystick.getRawAxis(OIConstants.kCodriverLTrigger)
                )
        );

        m_climber.setDefaultCommand(
                new ClimberJoystick(
                        m_climber,
                        () -> -codriverJoystick.getRawAxis(1)
                )
        );
    }

    /* ======================= */
    /* === BUTTON BINDINGS == */
    /* ======================= */
    private void configureButtonBindings() {

        /* ---------- DRIVER ---------- */
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value)
                .onTrue(new AutoAimSystem(swerveSubsystem, m_LimeLight,
                        () -> driverJoystick.getRawButton(6)));

        new JoystickButton(driverJoystick, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(swerveSubsystem::zeroHeading));

        new JoystickButton(driverJoystick, XboxController.Button.kY.value)
                .onTrue(new InstantCommand(() ->
                        swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

        // in RobotContainer.configureButtonBindings()


        new POVButton(driverJoystick, 90)
                .onTrue(new SwerveJogPosition(swerveSubsystem, 0,
                        Units.inchesToMeters(-6), () -> driverJoystick.getRawButton(7)));

        new POVButton(driverJoystick, 270)
                .onTrue(new SwerveJogPosition(swerveSubsystem, 0,
                        Units.inchesToMeters(6), () -> driverJoystick.getRawButton(7)));

        /* ---------- CODRIVER BUTTONS ---------- */
        // Conveyor Forward (X)
        new JoystickButton(codriverJoystick, OIConstants.kCpdriver_X)
                .whileTrue(Commands.run(m_conveyor::runConveyorDefault, m_conveyor));

        // Conveyor Reverse (B)
        new JoystickButton(codriverJoystick, OIConstants.kCodriver_B)
                .whileTrue(Commands.run(() ->
                        m_conveyor.runConveyor(-ConveyorConstants.kConveyorSpeed), m_conveyor));

        /* ---------- INTAKE BUTTONS ---------- */
        // Intake Forward (RB)
       // new JoystickButton(codriverJoystick, XboxController.Button.kRightBumper.value)
         //       .whileTrue(Commands.run(() -> m_fuel.runIntake(FuelIntakeConstants.kIntakeSpeed), m_fuel));

        // Intake Reverse (LB)
       // new JoystickButton(codriverJoystick, XboxController.Button.kLeftBumper.value)
         //       .whileTrue(Commands.run(() -> m_fuel.runIntake(-FuelIntakeConstants.kIntakeSpeed), m_fuel));
                        new JoystickButton(codriverJoystick, XboxController.Button.kRightStick.value)
    .whileTrue(Commands.run(() -> m_fuel.runIntake(FuelConstants.kFuelLoadGain), m_fuel));

        /* ---------- SHOOTER BUTTONS ---------- */
        // Shooter Forward (Right Trigger)
        new Trigger(() -> codriverJoystick.getRawAxis(OIConstants.kCodriverRTrigger)
                > ShooterConstants.kTriggerThreshold)
                .whileTrue(Commands.run(m_shooter::runShooterDefault, m_shooter));

        // Shooter Reverse (Left Trigger)
        new Trigger(() -> codriverJoystick.getRawAxis(OIConstants.kCodriverLTrigger)
                > ShooterConstants.kTriggerThreshold)
                .whileTrue(Commands.run(this::runShooterReverse, m_shooter));
    }

    /* ======================= */
    /* === HELPER METHODS === */
    /* ======================= */
    private void runShooterReverse() {
        m_shooter.runFlywheels(-ShooterConstants.kFlywheelSpeed);
        m_shooter.runIndexers(-ShooterConstants.kIndexerSpeed);
    }

    public void containerResetAllEncoders() {
        swerveSubsystem.resetAllEncoders();
    }

    /* ======================= */
    /* ===== AUTONOMOUS ===== */
    /* ======================= */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /* ======================= */
    /* === SHUFFLEBOARD ==== */
    /* ======================= */
    private void buildShuffleboard() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auton Select");
        autoTab.add("Auto Mode", autoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
        autoChooser.setDefaultOption(kDefaultAuto, new PathPlannerAuto(kDefaultAuto));
    }
}
