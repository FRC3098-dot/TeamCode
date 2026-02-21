package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeExtandConstants;

public class IntakeExtand extends SubsystemBase {
    private final SparkMax m_intakeExtandMotor;

    public IntakeExtand() {
        m_intakeExtandMotor = new SparkMax(IntakeExtandConstants.kIntakeExtandMotor, MotorType.kBrushless);
        // Optional configuration:
        // m_intakeExtandMotor.restoreFactoryDefaults();
        // m_intakeExtandMotor.setIdleMode(SparkMax.IdleMode.kBrake);
    }

    /** Run IntakeExtand at the configured default speed. */
    public void run() {
        run(IntakeExtandConstants.kIntakeExtandSpeed);
    }

    /** Run IntakeExtand at a specific speed (-1.0 to 1.0). */
    public void run(double speed) {
        m_intakeExtandMotor.set(speed);
    }

    /** Stop the IntakeExtand motor. */
    public void stop() {
        m_intakeExtandMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Optional periodic updates (telemetry, safety checks)
    }
}