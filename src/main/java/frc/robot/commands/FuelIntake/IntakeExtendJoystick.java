package frc.robot.commands.FuelIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.util.Constants.FuelConstants;

/**
 * Command that runs the fuel intake while held. When ended, it stops the intake.
 */
public class IntakeExtendJoystick extends Command {

    private final FuelIntakeSubsystem m_intake;

    public IntakeExtendJoystick(FuelIntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Run intake at the configured load speed (positive direction to extend/load)
        m_intake.runIntake(FuelConstants.kFuelLoadGain);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false; // run until released/interrupted
    }
}
