package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Constants.ShooterConstants;

/**
 * Command that runs the shooter (flywheels and indexers) while held.
 * Requires the ShooterSubsystem and uses configured default speeds from ShooterConstants.
 */
public class RunShooter extends Command {

    private final ShooterSubsystem m_shooter;

    public RunShooter(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // No-op
    }

    @Override
    public void execute() {
        // Run flywheels and indexers at configured default speeds
        m_shooter.runFlywheels(ShooterConstants.kFlywheelSpeed);
        m_shooter.runIndexers(ShooterConstants.kIndexerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter motors when command ends
        m_shooter.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted/cancelled
    }
}
