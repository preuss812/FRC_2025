// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.TrajectoryPlans;
import frc.robot.Utilities;

public class SwerveToProcessorCommand extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private AprilTag destination;
  private SequentialCommandGroup commands;

  
  /** Creates a new SwerveToProcesorComand.
   * This will drive the robot (hopefully) quickly and safely to the processor for the robot's alliance.
   */
  public SwerveToProcessorCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem
  ) {
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    // Next line is commented out because it is causing self cancellation
    // due to the FollowTrajectoryCommand being added to the scheduler.
    //addRequirements(robotDrive, poseEstimatorSubsystem);  // This may be a problem due to self cancellation.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose = poseEstimatorSubsystem.getCurrentPose();
    int allianceID = Utilities.getAllianceID();
  
    if (allianceID == FieldConstants.BlueAlliance) {
      destination = VisionConstants.AprilTag.BLUE_PROCESSOR;
    } else {
      destination = VisionConstants.AprilTag.RED_PROCESSOR;
    }

    List<Pose2d> waypoints = new ArrayList<>();
    Pose2d aprilTagPose = null;
    Pose2d nearTargetPose = null;
    //Pose2d targetPose = null;
    SmartDashboard.putString("TT","Running");
    //commands = new SequentialCommandGroup();

    if (destination == AprilTag.BLUE_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.BlueProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_PROCESSOR.id());
      nearTargetPose = DriveConstants.robotFrontAtPose(aprilTagPose, 0.0);
      waypoints.add(nearTargetPose);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.RED_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.RedProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.RED_PROCESSOR.id());
      nearTargetPose = DriveConstants.robotFrontAtPose(aprilTagPose, 0.0);
      waypoints.add(nearTargetPose);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else {
      //targetPose = startingPose; // This will end up doing nothing.
      nearTargetPose = startingPose;
    }
    //if (waypoints.size() > 0)
      Utilities.toSmartDashboard("TT Plan", waypoints);
    
    if (waypoints.size() > 0 && !startingPose.equals(nearTargetPose)) {
      commands = new FollowTrajectoryCommand(robotDrive, poseEstimatorSubsystem, null, waypoints);
      commands.schedule();
    }
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      commands.cancel();
    } catch (Exception e) {
    }
    SmartDashboard.putBoolean("TT Interrupted", interrupted);
    SmartDashboard.putString("TT","Done abort");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //boolean finished = true;
    return commands.isFinished() || !commands.isScheduled();
  }
}
