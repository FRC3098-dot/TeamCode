package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.Constants.ConveyorConstants;

/**
 * ConveyorSubsystem
 *
 * <p>Responsibility
 * - Controls a single motor that moves game pieces along an internal conveyor.
 * - Exposes simple open-loop control methods (run at specified speed, run at default
 *   configured speed, stop).
 *
 * <p>Hardware contract
 * - Uses a single REV SparkMax motor controller (CAN ID provided by
 *   {@link frc.robot.util.Constants.ConveyorConstants#kConveyorMotor}).
 * - Motor is configured to coast when idle by default (see constructor). If your
 *   mechanism needs braking, change IdleMode to BRAKE.
 *
 * <p>Behavior and safety
 * - All methods are non-blocking and return immediately.
 * - This subsystem publishes basic telemetry (current output) to Shuffleboard/SmartDashboard
 *   under the keys "Conveyor Motor Speed".
 * - This implementation uses open-loop voltage control (set motor to a fraction -1..1).
 *   Consider replacing with a closed-loop velocity controller if you need consistent
 *   speeds under varying load.
 * - If you have sensors (beam break, optical, or limit switch) that detect pieces,
 *   prefer adding them and writing higher-level commands that coordinate the conveyor
 *   with the shooter/indexers to prevent jams.
 *
 * <p>Concurrency
 * - Commands that require this subsystem will gain exclusive control of the motor via
 *   the scheduler. Use the {@code requires()} / default command patterns to avoid
 *   conflicting control.
 *
 * <p>Tuning & testing
 * - Default conveyor speed is {@link frc.robot.util.Constants.ConveyorConstants#kConveyorSpeed}.
 * - Test on the robot with low speeds first to verify direction and clearance.
 * - If the conveyor runs the opposite direction of what you expect, set
 *   {@code m_config.inverted(true)} or invert the wiring depending on your preference.
 */

public class ConveyorSubsystem extends SubsystemBase {

    private final SparkMax m_conveyorMotor = new SparkMax(ConveyorConstants.kConveyorMotor, MotorType.kBrushless);
    /** Configuration object used to set motor idle mode, inversion, and current limits. */
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    public ConveyorSubsystem() {
        // Basic motor configuration. These default values are reasonable starting
        // points: coast when idle, not inverted, and modest current limiting to
        // protect the gearbox and motor from prolonged stalls.
        m_config.idleMode(IdleMode.kCoast);
        m_config.inverted(false);
        m_config.smartCurrentLimit(30);

        // Apply the configuration if the REV library version supports it. Older
        // library versions may not expose the configure(...) API; we catch
        // NoSuchMethodError to retain compatibility with multiple REV versions.
        try {
            m_conveyorMotor.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); // safe to ignore if missing
        } catch (NoSuchMethodError e) {
            // Older REV libs may not have configure; ignore if unavailable.
        }
    }

    @Override
    public void periodic() {
        // Publish the motor output for debugging. This is the last commanded
        // motor output (not a closed-loop velocity). Useful when tuning speeds
        // or diagnosing jams.
        SmartDashboard.putNumber("Conveyor Motor Speed", m_conveyorMotor.get());
    }

    /**
     * Run the conveyor at the given speed (-1..1).
    
     * @param speed Motor output where 1.0 is full forward and -1.0 is full reverse.
     *              Use small values for testing (0.2-0.6) until you verify direction
     *              and feed behavior.
     */
    public void runConveyor(double speed) {
        m_conveyorMotor.set(speed);
    }

    /**
     * Convenience helper that runs the conveyor at the default configured speed.
     * Uses {@link frc.robot.util.Constants.ConveyorConstants#kConveyorSpeed}.
     *
     * This method is useful for simple operator controls (e.g. hold-to-run) or
     * tests where a fixed feed speed is desired.
     */
    public void runConveyorDefault() {
        runConveyor(ConveyorConstants.kConveyorSpeed);
    }

    /** Stop the conveyor immediately by setting motor output to zero. */
    public void stopConveyor() {
        m_conveyorMotor.set(0);
    }
}
