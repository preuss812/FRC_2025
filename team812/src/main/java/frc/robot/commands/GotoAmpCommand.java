// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This command drives the robot to the AMP for the robot's alliance.
 * It will eventually follow a pre-planned trajectory from anywhere on the
 * field to the AMP.  It should probably lower the arm and quiese the intake
 * shooter motors as well.
 * The anywhere on the field part is not implemented here yet as it has some issues.
 */
public class GotoAmpCommand extends GotoPoseCommand {
  /** Creates a new GotoAmpCommand. */
  private final boolean debug = false;
  public GotoAmpCommand(PoseEstimatorSubsystem PoseEstimatorSubsystem
    , DriveSubsystemSRX DriveSubsystemSRXSubsystem) { 
    super(PoseEstimatorSubsystem,DriveSubsystemSRXSubsystem, new Pose2d()); // Pose is a place holder.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    xController = new PIDController(
      m_config.getLinearP(),
      m_config.getLinearI(),
      m_config.getLinearD()
    );
    xController.setIZone(m_config.getLinearIZone());
    yController = new PIDController(
      m_config.getLinearP(),
      m_config.getLinearI(),
      m_config.getLinearD()
    );
    yController.setIZone(m_config.getAngularIZone()); // TODO Needs Tuning.

    rotationController = new PIDController(
      m_config.getAngularP(),
      m_config.getAngularI(),
      m_config.getAngularD()
      );
    rotationController.setTolerance(m_config.getAngularTolerance()); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians).
    onTarget = false;

    if (Utilities.isBlueAlliance()) {
      Pose2d tag = m_PoseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.BLUE_AMP.id());
      // This should position the robot back to the AMP touching the wall.
      targetPose = new Pose2d(tag.getX(), tag.getY() - DriveConstants.kBackToCenterDistance, tag.getRotation());

    } else if (Utilities.isRedAlliance()) {
      Pose2d tag = m_PoseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.RED_AMP.id());
      // This should position the robot back to the AMP touching the wall.
      targetPose = new Pose2d(tag.getX(), tag.getY() - DriveConstants.kBackToCenterDistance, tag.getRotation());
    }
    else {
      targetPose = m_PoseEstimatorSubsystem.getCurrentPose(); // Hack:: if we dont know the alliance. Dont move. 
    }

    if (true) Utilities.toSmartDashboard("GotoTarget", targetPose);
    if (debug) SmartDashboard.putBoolean("GotoPoseOnTarget", false); // We will need to check in execute
  }
}
