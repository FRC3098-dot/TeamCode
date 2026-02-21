// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.Constants.FuelConstants;

/**
 * FuelIntakeSubsystem
 *
 * Controls the fuel/intake motor and limit switch. Methods are intentionally small and
 * named for clarity so commands can call them directly.
 */
@SuppressWarnings("unused")
public class FuelIntakeSubsystem extends SubsystemBase {

    // Motor controller for the intake
    private final SparkMax m_intakeMotor = new SparkMax(FuelConstants.kFuelIntake, MotorType.kBrushed);

    // Configuration object used to set idle mode, inversion, current limit, etc.
    private final SparkMaxConfig m_intakeMotorConfig = new SparkMaxConfig();

    // Limit switch to detect presence/position of fuel
    private final DigitalInput m_intakeLimit = new DigitalInput(FuelConstants.kFuelIntakeLimit);

    /** Constructor: configure motor parameters here. */
    public FuelIntakeSubsystem() {
        // Configure motor defaults
        m_intakeMotorConfig.idleMode(IdleMode.kBrake);
        m_intakeMotorConfig.inverted(true);
        m_intakeMotorConfig.smartCurrentLimit(40);

        // Apply configuration to the motor controller. If the configure method is
        // not available in your REV library, comment the line below.
        try {
            m_intakeMotor.configure(m_intakeMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
        } catch (NoSuchMethodError e) {
            // Some versions of the library may not expose configure; ignore if unavailable.
        }
    }

    @Override
    public void periodic() {
        // Update telemetry periodically
        updateSmartDashboard();
    }

    // ---------------- Intake control methods ----------------

    /** Run intake at the given speed. Positive/negative should match wiring and convention. */
    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    /** Run intake for feeding (example: pull in cargo). Kept as a separate method for clarity. */
    public void runFeed(double speed) {
        m_intakeMotor.set(speed);
    }

    /** Run intake for loading (example: push out cargo). */
    public void runLoad(double speed) {
        m_intakeMotor.set(speed);
    }

    /** Stop intake motor immediately. */
    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    /** Returns the raw limit switch state. True/false depends on wiring (closed vs open). */
    public boolean isLimitPressed() {
        return m_intakeLimit.get();
    }

    /** Puts basic telemetry to SmartDashboard for debugging. */
    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Fuel Intake Limit", m_intakeLimit.get());
    }

    // Compatibility wrappers for older command names (keeps existing commands working).
    /** Backwards-compatible wrapper used by older commands to run the intake at a steady speed. */
    public void toggleFuelIntake(double speed) {
        runIntake(speed);
    }

    /** Backwards-compatible wrapper used by older commands to run the intake in load mode. */
    public void toggleFuelIntakeLoad(double speed) {
        runLoad(speed);
    }

    /** Backwards-compatible wrapper used by older commands to run the intake in feed mode. */
    public void toggleFuelIntakeFeed(double speed) {
        runFeed(speed);
    }
}
