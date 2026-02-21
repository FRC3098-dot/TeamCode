// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * AutoAimSystem
 *
 * <p>Purpose
 * - Provides a command that uses Limelight vision data to automatically drive and rotate
 *   a swerve-drive robot toward a vision target. It computes chassis speeds (vx, vy, omega)
 *   using PID controllers and slew-rate limiters, then delegates low-level actuation to the
 *   {@code SwerveSubsystem}.
 *
 * <p>Public API (constructor)
 * - AutoAimSystem(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem,
 *                 Supplier<Boolean> rtBumperSupplier)
 *
 * <p>Parameters
 * - swerveSubsystem: swerve drive subsystem to command (required).
 * - limelightSubsystem: wrapper for Limelight sensor interactions (required).
 * - rtBumperSupplier: supplier used to determine when to continue/stop the command (eg
 *   reading a driver trigger). When the supplier returns false the command finishes.
 *
 * <p>Behavior & implementation notes
 * - Reads Limelight TX/TY values and computes a distance estimate using camera mounting
 *   angle and known target height (see {@code VisionConstants}).
 * - Converts the camera offsets into world-relative X/Y errors and runs PID controllers to
 *   generate target chassis velocities. Values are limited with {@code SlewRateLimiter}s
 *   and scaled by {@code DriveConstants.kTeleDriveMaxSpeedMetersPerSecond}.
 * - Rotation control goes through a small PID controller and is scaled by
 *   {@code DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond}.
 *
 * <p>Side effects
 * - Requires the swerve and limelight subsystems and will command robot motion.
 * - Forces Limelight LED modes when a target is acquired.
 *
 * <p>Tuning and safety
 * - PID gains and limiter rates are sensitive; tune conservatively on robot at low speed.
 * - The underlying subsystems must enforce hardware limits (speed caps, watchdogs, limit
 *   switches) — this command does not bypass subsystem safety.
 *
 * <p>Edge cases
 * - If no valid target is present (tv == false) the command stops the modules and turns
 *   off the Limelight LEDs.
 */
package frc.robot.commands.Routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.VisionConstants;
// Removed unused reef score pose import (Reefs constants removed).
import frc.robot.util.LimelightHelpers;
import frc.robot.util.BlinkinLEDController;
import frc.robot.util.Constants;
import frc.robot.util.BlinkinLEDController.BlinkinPattern;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.*;

// Ignore unused variable warnings
@SuppressWarnings("unused")

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimSystem extends Command{
  
   private final SwerveSubsystem swerveSubsystem;
   private final LimelightSubsystem m_LimelightSubsystem;
   private final SlewRateLimiter xLimiter= new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
   private final SlewRateLimiter  yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond); 
   private final SlewRateLimiter turningLimiter= new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
   private PIDController vXController =new PIDController(0.5, 0.005, 0);//was 1.8 
   private PIDController vYController=new PIDController(0.5, 0.005, 0);//was 1.8,
   private PIDController  LLthetaController=new PIDController(0.1, 0, 0);
  
   private double turningSpeed, xLimeSpeed,yLimeSpeed,ty,tx;
   private double xLimeError,yLimeError, distanceToTarget, AprilTagID;
   private boolean TargetValid;
   private Pose2d startingPose;
   private ChassisSpeeds chassisSpeeds;
   private BlinkinLEDController ledValue;
   private final Supplier<Boolean> RtBumper;



  public AutoAimSystem(SwerveSubsystem swerveSubsystem, LimelightSubsystem m_LimelightSubsystem, Supplier<Boolean> RtBumper){
    
    this.swerveSubsystem = swerveSubsystem;
    this.m_LimelightSubsystem =m_LimelightSubsystem;
    this.RtBumper = RtBumper;
    addRequirements(swerveSubsystem,m_LimelightSubsystem);
    LLthetaController.enableContinuousInput(0, 360);
   
   }
   @Override
   public void initialize() {
    swerveSubsystem.resetOdometry(new Pose2d(0,0,new Rotation2d()));
    startingPose = swerveSubsystem.getPose(); // Gets inital X & Y Position 
    AprilTagID = LimelightHelpers.getFiducialID(VisionConstants.CameraName);
   }

   @Override
   public void execute() {
     if (LimelightHelpers.getTV(VisionConstants.CameraName)) {
          LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.CameraName);
          chassisSpeeds= getLimeLightvalues();
          swerveSubsystem.drive(chassisSpeeds,false);
       } 
     else {
           swerveSubsystem.stopModules();
           LimelightHelpers.setLEDMode_ForceOff(VisionConstants.CameraName);
          }

      LimeSmartboardUpdate();
   }
   @Override
   public boolean isFinished() {
    return !RtBumper.get();
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

public ChassisSpeeds getLimeLightvalues(){
//Get Limelight Values
 ty = LimelightHelpers.getTY(VisionConstants.CameraName); 
 tx = LimelightHelpers.getTX(VisionConstants.CameraName);

 //Compute Distance to Target
 distanceToTarget = (VisionConstants.hTarget-VisionConstants.hCamera)/Math.tan(Units.degreesToRadians(VisionConstants.cameraMountAngle + ty));
 
 //Convert TX to real-world X position
 xLimeError=distanceToTarget * Math.tan(Math.toRadians(tx));
 yLimeError=distanceToTarget;

 //Note: Plus sign is due to camera data being negative relative to robot positions
 // xLimeError = ( LimelightHelpers.getTY(VisionConstants.CameraName) - (swerveSubsystem.getPose().getX() - startingPose.getX()));
 // yLimeError = ( LimelightHelpers.getTX(VisionConstants.CameraName)-(swerveSubsystem.getPose().getY() - startingPose.getY()));
  

  xLimeSpeed = vXController.calculate(xLimeError,0.0); // Define X-Axis Setpoint 
  yLimeSpeed = vYController.calculate(yLimeError, 0.0); // Define Y-Axis Setpoint
  turningSpeed = LLthetaController.calculate(Math.toRadians(swerveSubsystem.getHeading()), 0); // define Rotation Setpoint
 
  xLimeSpeed = xLimiter.calculate(xLimeSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
  yLimeSpeed = yLimiter.calculate(yLimeSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
  turningSpeed = -turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
  
  return chassisSpeeds = new ChassisSpeeds(xLimeSpeed,yLimeSpeed,turningSpeed);
}

public void LimeSmartboardUpdate(){
    SmartDashboard.putBoolean("Target Valid", LimelightHelpers.getTV(VisionConstants.CameraName));
    SmartDashboard.putNumber("Tag ID", LimelightHelpers.getFiducialID(VisionConstants.CameraName));
    SmartDashboard.putNumber("Distance to Target", distanceToTarget);

    SmartDashboard.putNumber("Get Heading(Degrees)", swerveSubsystem.getHeading());
    SmartDashboard.putNumber("X Starting Position", startingPose.getX());
    SmartDashboard.putNumber("vX", yLimeSpeed);
    SmartDashboard.putNumber("Y Starting Position", startingPose.getY());
    SmartDashboard.putNumber("vY", xLimeSpeed);
    SmartDashboard.putNumber("vT", turningSpeed);
    SmartDashboard.putNumber("X Lime Error",  LimelightHelpers.getTY(VisionConstants.CameraName) + (swerveSubsystem.getPose().getX() - startingPose.getX()));
    SmartDashboard.putNumber("Y Lime Error", LimelightHelpers.getTX(VisionConstants.CameraName) + (swerveSubsystem.getPose().getY() - startingPose.getY()));
    SmartDashboard.putString("Camera Name",VisionConstants.CameraName);
    SmartDashboard.putNumber("April Tag", AprilTagID);
    SmartDashboard.putNumber("TY", LimelightHelpers.getTY(VisionConstants.CameraName));
    SmartDashboard.putNumber("TX",LimelightHelpers.getTX(VisionConstants.CameraName));
    SmartDashboard.putBoolean("Tv", LimelightHelpers.getTV(VisionConstants.CameraName));
  }
} 