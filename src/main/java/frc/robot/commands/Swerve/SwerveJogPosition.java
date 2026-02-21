// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

//Math Libraries
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

//SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//Robot Commands & Subsystems
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.LimelightHelpers;

//Library Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Trajectory - creating custom Paths
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class SwerveJogPosition extends Command {

  private SwerveSubsystem swerveSubsystem;
  private double Xpose, Ypose, speed, turningSpeed, xError,yError, xSpeed,ySpeed, initialHeading,
                       xCurrent, yCurrent,targetDistance, distanceTraveled;
  private double Xtarget=0;
  private double Ytarget=0;
  private Pose2d initialPose;
  private boolean m_fieldOrientation =false;
  private ChassisSpeeds chassisSpeeds;
  private final SlewRateLimiter xLimiter= new SlewRateLimiter(1);
  private final SlewRateLimiter  yLimiter = new SlewRateLimiter(1); 
  private final SlewRateLimiter turningLimiter= new SlewRateLimiter(1);
  //TODO: Adjust KP for both vX and vY
  private PIDController vXController =new PIDController(2.0, 0.5, 0);//was 1.8,0.5
  private PIDController vYController=new PIDController(2.0, 0.05, 0);//was 1.8,0.5
  private PIDController thetaController= new PIDController(0.25, 0, 0);
  private final Supplier<Boolean> BackButton; 
 
        
  public SwerveJogPosition(SwerveSubsystem swerveSubsystem, double Xpose, double Ypose, Supplier<Boolean> BackButton){
   this.swerveSubsystem = swerveSubsystem;
   this.Xpose=Xpose;
   this.Ypose=Ypose;
   this.BackButton = BackButton;
   this.m_fieldOrientation=false;
   addRequirements(swerveSubsystem);
  }    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   swerveSubsystem.resetOdometry(new Pose2d(0,0,new Rotation2d()));
   initialPose = swerveSubsystem.getPose(); // Gets X & Y Position, Translational   
   initialHeading = swerveSubsystem.getHeading();
  
   Xtarget = initialPose.getX()+Xpose;
   Ytarget = initialPose.getY()+Ypose;
   
   targetDistance = Math.sqrt(Math.pow(Xtarget, 2) + Math.pow(Ytarget, 2));
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSpeeds=Jogswervedrive();
    JogSmartDashboardupdate();
    swerveSubsystem.drive(chassisSpeeds,m_fieldOrientation);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
 public boolean isFinished() {
  //TODO:Uncomment statement if system runs continuously
  //if(distanceTraveled>=targetDistance)
  if((targetDistance-distanceTraveled<0.02) || (BackButton.get())){
    swerveSubsystem.resetOdometry(new Pose2d(0,0,new Rotation2d()));
    return true;}
  else
    return false;
  }

public ChassisSpeeds Jogswervedrive(){
  xCurrent=swerveSubsystem.getPose().getX();
  yCurrent= swerveSubsystem.getPose().getY();

  xError = (Xtarget-xCurrent);
  yError = (Ytarget-yCurrent);

 // Calculate total distance traveled using Pythagorean theorem
  distanceTraveled = Math.sqrt(Math.pow(xCurrent, 2) + Math.pow(yCurrent, 2));

  xSpeed = vXController.calculate(xError,0); // Define X-Axis Setpoint 
  ySpeed = vYController.calculate(yError,0); // Define Y-Axis Setpoint
  turningSpeed = thetaController.calculate(Units.degreesToRadians(initialHeading), Units.degreesToRadians(swerveSubsystem.getHeading())); // define Rotation Setpoint

  xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
  ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
  turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
 
  return chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);
}

public void JogSmartDashboardupdate(){
  SmartDashboard.putNumber("Xpose",Xpose);
  SmartDashboard.putNumber("Ypose",Ypose);
  SmartDashboard.putNumber("X Target",Xtarget);
  SmartDashboard.putNumber("Y Target",Ytarget);
  SmartDashboard.putNumber("StartingPoseX",initialPose.getX());
  SmartDashboard.putNumber("StartingPoseY",initialPose.getY());
  SmartDashboard.putNumber("Xheading", xCurrent); 
  SmartDashboard.putNumber("Yheading", yCurrent);
  SmartDashboard.putNumber("xError", xError);
  SmartDashboard.putNumber("yError",yError);
  SmartDashboard.putNumber("xSpeed", xSpeed);
  SmartDashboard.putNumber("ySpeed",ySpeed);
  SmartDashboard.putNumber("turningSpeed",turningSpeed);
  SmartDashboard.putNumber("Distance Traveled",distanceTraveled);
  SmartDashboard.putNumber("Target Distance",targetDistance);
  SmartDashboard.putBoolean("Field Oriented",m_fieldOrientation);
}

}

