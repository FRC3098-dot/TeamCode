// RobotBuilder Version: 4.0
package frc.robot.commands.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.Supplier;

import frc.robot.util.Constants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ClimberConstants;

import java.lang.Math;
import frc.robot.RobotContainer;
// Ignore unused variable warnings
@SuppressWarnings("unused")

/** Command to control the climber with a joystick axis. */
public class ClimberJoystick extends Command {

    private final ClimberSubsystem m_climber;
    private Supplier<Double> ClimberSpd;
    private SlewRateLimiter ClimberSlew = new SlewRateLimiter(ClimberConstants.kElMotorSlew);

    public ClimberJoystick(ClimberSubsystem subsystem, Supplier<Double> ClimberSpd) {
       this.ClimberSpd = ClimberSpd;
       this.m_climber = subsystem;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       double Vspeed = ClimberSpd.get();
       Vspeed = Math.abs(Vspeed) > ClimberConstants.kElDeadband ? Vspeed: 0.0;
       Vspeed = ClimberSlew.calculate(Vspeed)*Constants.ClimberConstants.kElMotorSlew;
       m_climber.setClimberPoseManual(Vspeed*Constants.ClimberConstants.kElMotorGain);
    }
    @Override
    public void end(boolean interrupted) {
        m_climber.setClimberPoseZero();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
