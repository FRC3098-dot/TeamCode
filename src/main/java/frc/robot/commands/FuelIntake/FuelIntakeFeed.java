// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.FuelIntake;

import java.lang.Math;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.util.Constants.*;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;


// Ignore unused variable warnings
@SuppressWarnings("unused")

/** An example command that uses an example subsystem. */
public class FuelIntakeFeed extends Command {

private FuelIntakeSubsystem m_fuelIntake;

public FuelIntakeFeed(FuelIntakeSubsystem m_fuelIntake) {
   
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_fuelIntake = m_fuelIntake;
    addRequirements(m_fuelIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_fuelIntake.toggleFuelIntakeFeed(FuelConstants.kFuelUnloadGain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_fuelIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
