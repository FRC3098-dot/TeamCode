package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberZeroHeight extends Command {

    private final ClimberSubsystem subsystem;

    public ClimberZeroHeight(ClimberSubsystem subsystem) {
       this.subsystem = subsystem;
       addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.setClimberPose(0.0);
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {
        subsystem.stopClimber();
    }
    @Override
    public boolean isFinished() {
        return Math.abs(subsystem.getClimberPosition() - 0.0) < 0.01;
    }
    @Override
    public boolean runsWhenDisabled() { return false; }
}
