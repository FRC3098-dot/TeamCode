// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.FuelIntake;

import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.util.Constants.*;
import frc.robot.RobotContainer;

import java.util.function.Supplier;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;


// Ignore unused variable warnings
@SuppressWarnings("unused")

/** An example command that uses an example subsystem. */
public class FuelIntakeLoad extends Command {

  private FuelIntakeSubsystem m_Fuelintake;
   
public FuelIntakeLoad(FuelIntakeSubsystem m_fuelintake) {
   
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Fuelintake = m_fuelintake;
    addRequirements(m_fuelintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
  m_Fuelintake.toggleFuelIntakeLoad(FuelConstants.kFuelLoadGain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_Fuelintake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
