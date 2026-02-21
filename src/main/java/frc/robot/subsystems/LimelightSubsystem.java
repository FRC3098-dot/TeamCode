// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /**
   * Camera network-table name constants are defined in {@link frc.robot.util.Constants.VisionConstants}.
   * This subsystem will query each configured camera and choose an active one when a valid
   * target is found.
   */

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTx(String limelightName) {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTy(String limelightName) {
    return LimelightHelpers.getTY(limelightName);
  }

  public boolean hasValidTarget(String limelightName) {
    return LimelightHelpers.getTV(limelightName);
  }

  /**
   * Returns the name of the first camera (priority order: front, back, top) that has a
   * valid target. Returns null if none are valid.
   */
  public String getActiveCamera() {
    if (hasValidTarget(frc.robot.util.Constants.VisionConstants.CameraFront)) {
      return frc.robot.util.Constants.VisionConstants.CameraFront;
    } else if (hasValidTarget(frc.robot.util.Constants.VisionConstants.CameraBack)) {
      return frc.robot.util.Constants.VisionConstants.CameraBack;
    } else if (hasValidTarget(frc.robot.util.Constants.VisionConstants.CameraTop)) {
      return frc.robot.util.Constants.VisionConstants.CameraTop;
    }
    return null; // No active camera
  }

  /** Returns true if any configured camera currently reports a valid target. */
  public boolean hasTarget() {
    return hasValidTarget(frc.robot.util.Constants.VisionConstants.CameraFront)
        || hasValidTarget(frc.robot.util.Constants.VisionConstants.CameraBack)
        || hasValidTarget(frc.robot.util.Constants.VisionConstants.CameraTop);
  }

  /**
   * Compute a distance estimate to the target using the camera mounting values in
   * {@link frc.robot.util.Constants.VisionConstants}.
   *
   * @param limelightName camera network-table name to use for the calculation
   * @return distance in meters (may be NaN or Infinite if invalid readings occur)
   */
  public double getDistance(String limelightName) {
    double targetHeight = frc.robot.util.Constants.VisionConstants.hTarget;
    double cameraHeight = frc.robot.util.Constants.VisionConstants.hCamera;
    double cameraAngle = frc.robot.util.Constants.VisionConstants.cameraMountAngle;

    double ty = getTy(limelightName);
    double angleToTarget = Math.toRadians(cameraAngle + ty);
    return (targetHeight - cameraHeight) / Math.tan(angleToTarget);
  }

  public void setLEDMode(String limelightName) {
    LimelightHelpers.setLEDMode_ForceOn(limelightName);
  }

  public void setCamMode(String limelightName) {
    LimelightHelpers.setCameraMode_Processor(limelightName);
  }
}
