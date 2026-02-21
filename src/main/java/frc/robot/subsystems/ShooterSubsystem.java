package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.Constants.ShooterConstants;

/**
 * ShooterSubsystem
 *
 * <p>Overview
 * - Controls two flywheel motors (top and bottom) that provide the projectile
 *   launch velocity, and two indexer motors that feed projectiles into the
 *   flywheels.
 * - This implementation is intentionally simple and uses open-loop control
 *   (motor.set(speed)). For consistent shot velocities consider adding
 *   closed-loop velocity control (PID + feedforward) on the flywheels.
 *
 * <p>Design notes and recommendations
 * - In many mechanical layouts the two flywheels spin in opposite directions
 *   (one motor must be inverted). The code in {@link #runFlywheels(double)} inverts
 *   the right-side motor by setting the opposite sign — if your wiring differs,
 *   change the inversion or wiring accordingly.
 * - Indexers should only run once the flywheels are up to speed to avoid jams
 *   and shot inconsistency. Consider adding a 'ready' check using RPM if you
 *   implement closed-loop flywheel control.
 * - Current limiting is enabled in the constructor via SparkMaxConfig to help
 *   protect motors and mechanicals during jams.
 *
 * <p>Safety
 * - This subsystem exposes open-loop methods. Higher-level commands should
 *   enforce safety interlocks (for example: do not run indexers without flywheels
 *   at target speed).
 * - Telemetry is published in {@link #periodic()} to help diagnose behavior.
 *
 * <p>Tuning & testing
 * - Start with low flywheel speeds on the real robot (0.2-0.5) and verify
 *   direction and that the shot trajectory is reasonable before increasing.
 * - If you migrate to closed-loop control use an encoder on at least one
 *   flywheel and tune a PID controller with feedforward for stable velocity.
 */

public class ShooterSubsystem extends SubsystemBase {

    /** Top flywheel motor controller. */
    private final SparkMax m_flyFront = new SparkMax(ShooterConstants.kFlywheelFront, MotorType.kBrushless);

    /** Bottom flywheel motor controller. Often mounted mirrored; sign is inverted when running. */
    private final SparkMax m_flyBack = new SparkMax(ShooterConstants.kFlywheelBack, MotorType.kBrushless);

    /** Indexer motors that push projectiles into the flywheel intake. */
    private final SparkMax m_indexFront = new SparkMax(ShooterConstants.kIndexerFront, MotorType.kBrushless);
    private final SparkMax m_indexBack = new SparkMax(ShooterConstants.kIndexerBack, MotorType.kBrushless);
    private final SparkMax m_indexStar = new SparkMax(ShooterConstants.kIndexerStar, MotorType.kBrushless);

    /** Configuration object shared for basic motor settings (idle mode, current limit). */
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    public ShooterSubsystem() {
        m_config.idleMode(IdleMode.kCoast);
        m_config.inverted(false);
        m_config.smartCurrentLimit(40);

        // Apply configuration to each motor. If the REV library doesn't expose
        // the configure API on this version, the NoSuchMethodError is caught and
        // the subsystem will still function (without persisted configuration).
        try {
            m_flyFront.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
            m_flyBack.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
            m_indexFront.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
            m_indexBack.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
            m_indexStar.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        } catch (NoSuchMethodError e) {
            // Older REV libraries may not have configure; ignore if unavailable.
        }
    }

    @Override
    public void periodic() {
        // Publish the last commanded outputs for each motor. Note: these are
        // open-loop outputs and not measured RPMs. Replace with sensor feedback
        // if you need closed-loop monitoring.
        SmartDashboard.putNumber("FlyFront speed", m_flyFront.get());
        SmartDashboard.putNumber("FlyBack speed", m_flyBack.get());
        SmartDashboard.putNumber("IndexFront speed", m_indexFront.get());
        SmartDashboard.putNumber("IndexBack speed", m_indexBack.get());
        SmartDashboard.putNumber("IndexStar speed", m_indexStar.get());
    }

    /**
     * Run both flywheels at the given speed (-1..1).
     *
     * <p>One side may need inverted output depending on motor mounting. The
     * implementation inverts the right flywheel when commanding speed to make
     * it straightforward to call this method with a single sign for "shoot
     * forward". If your hardware is wired differently, change the inversion
     * here or via SparkMax configuration.
     *
     * @param speed desired motor output (1.0 full forward, -1.0 full reverse)
     */
    public void runFlywheels(double speed) {
        m_flyFront.set(speed);
        // Invert right-side output to account for mirrored mounting; adjust if needed.
        m_flyBack.set(-speed);
    }

    /**
     * Run both indexers at the given speed (-1..1). Indexers should typically
     * run more slowly than flywheels and only when flywheels are at target
     * velocity to avoid crushing or jamming pieces.
     *
     * @param speed desired motor output for indexers
     */
    public void runIndexers(double speed) {
        m_indexFront.set(speed);
        m_indexBack.set(speed);
        m_indexStar.set(speed);
    }

    /**
     * Convenience method to start the shooter using default speeds. This starts
     * both flywheels and indexers. For safer operation, prefer starting
     * flywheels first and only enabling indexers after the flywheels are
     * confirmed to be up to speed (if you implement RPM monitoring).
     */
    public void runShooterDefault() {
        runFlywheels(ShooterConstants.kFlywheelSpeed);
        runIndexers(ShooterConstants.kIndexerSpeed);
    }

    /** Stop all shooter motors immediately. */
    public void stopAll() {
        runFlywheels(0);
        runIndexers(0);
    }
}
