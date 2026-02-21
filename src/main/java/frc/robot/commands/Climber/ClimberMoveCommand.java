package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMoveCommand extends Command {
  
  private final ClimberSubsystem climberSubsystem;
  private final double targetheight;
  private final double duration;
  private Timer timer = new Timer();

  public ClimberMoveCommand(ClimberSubsystem subsystem, double targetheight, double duration) {
    this.climberSubsystem = subsystem;
    this.targetheight = targetheight;
    this.duration = duration;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    climberSubsystem.setClimberPose(targetheight);
    SmartDashboard.putNumber("Climber target height", targetheight);
  }

  @Override
  public void end(boolean interrupted) {
   climberSubsystem.stopClimber();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= duration;
  }
}
