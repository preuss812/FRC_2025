// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This command drives the robot to the SOURCE for the robot's alliance.
 * It will eventually follow a pre-planned trajectory from anywhere on the
 * field to the SOURCE.  It should probably lower the arm and quiese the intake
 * shooter motors as well.
 */
public class SwerveToAmpCommand extends SwerveToPoseCommand {
  /** Creates a new SwerveToAmpCommand. */
  public SwerveToAmpCommand(
    DriveSubsystemSRX DriveSubsystemSRXSubsystem,
    PoseEstimatorSubsystem PoseEstimatorSubsystem
  ) { 
    // Constructor picks the alliance tag and the rest is standard SwerveToPoseCommand.
    // Probably should have been named SwerveToTagCommand.
    super(
      DriveSubsystemSRXSubsystem,
      PoseEstimatorSubsystem,
      Utilities.isBlueAlliance() ? VisionConstants.AprilTag.BLUE_AMP : VisionConstants.AprilTag.BLUE_AMP);
  }

  @Override
  public void initialize() {
    this.destination = Utilities.isBlueAlliance() ? VisionConstants.AprilTag.BLUE_AMP : VisionConstants.AprilTag.RED_AMP;
    super.initialize();
  }

}