// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.FuelIntake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Constants.FuelConstants;
import frc.robot.util.Constants.ClimberConstants;
import frc.robot.RobotContainer;
import java.lang.Math;
import java.util.function.Supplier;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.filter.SlewRateLimiter;

// Ignore unused variable warnings
@SuppressWarnings("unused")

/** An example command that uses an example subsystem. */
public class FuelIntakeJoystick extends Command {

 private FuelIntakeSubsystem m_Fuelintake;
 private Supplier<Double> RtTrigger, LtTrigger;
 //private double intakeLoadSpd, intakeFeedSpd;
 private SlewRateLimiter FuelIntakeLimiter;
   
 public FuelIntakeJoystick(FuelIntakeSubsystem m_Fuelintake, Supplier<Double> RtTrigger, Supplier<Double> LtTrigger) {
   
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Fuelintake = m_Fuelintake;
    this.RtTrigger=RtTrigger;
    this.LtTrigger=LtTrigger;

     addRequirements(m_Fuelintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  double rightTrigger = Math.abs(RtTrigger.get())>FuelConstants.kFuelDeadband ? RtTrigger.get()  : 0.0;
  double leftTrigger = Math.abs(LtTrigger.get())>FuelConstants.kFuelDeadband ? LtTrigger.get()  : 0.0;

  if(leftTrigger >0.1)
        m_Fuelintake.toggleFuelIntake(FuelConstants.kFuelLoadGain);
  else if(rightTrigger >0.1){
      m_Fuelintake.toggleFuelIntake(FuelConstants.kFuelUnloadGain); }
  else 
     m_Fuelintake.stopIntake();
  
  //Depricated this method
     /*double intakeSpeed = leftTrigger-rightTrigger; //Left for Load, Right for Feed
   //intakeSpeed = FuelIntakeLimiter.calculate(intakeSpeed) *Constants.ClimberConstants.kElMotorSlew;
  //m_fuelIntake.toggleFuelIntake(intakeSpeed);
  SmartDashboard.putNumber("Intake Speed",intakeSpeed);
  SmartDashboard.putNumber("Lt Trigger",LtTrigger.get());
  SmartDashboard.putNumber("Rt Trigger",RtTrigger.get());*/
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
