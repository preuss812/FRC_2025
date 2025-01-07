// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryPlans;
import frc.robot.Utilities;

public class SwerveToPoseTest extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private AprilTag destination;
  private SequentialCommandGroup commands;
  /**
   * For debug,
   * Remember field location and alliance to displaying trajactories
   * 
   */
  private static int i = 0;
  private static int j = 0;
  private static int allianceID = FieldConstants.BlueAlliance;
  private static int destinationID = 1; // 0=amp; 1 = source;
  
  /** Creates a new SwerveToPoseTest. */
  public SwerveToPoseTest(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem
  ) {
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(robotDrive, poseEstimatorSubsystem);  // This may be a problem due to self cancellation.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose;

    if (allianceID == FieldConstants.BlueAlliance) {
      if (destinationID == 0) {
        destination = VisionConstants.AprilTag.BLUE_AMP;
        startingPose = new Pose2d(TrajectoryPlans.BlueAmpPlan.plan[i][j].waypoint, new Rotation2d(-Math.PI/2.0));
      } else {
        destination = VisionConstants.AprilTag.BLUE_RIGHT_SOURCE;
        startingPose = new Pose2d(TrajectoryPlans.BlueAmpPlan.plan[i][j].waypoint, new Rotation2d(Math.PI/2.0));
      }
    } else {
      if (destinationID == 0) {
        destination = VisionConstants.AprilTag.RED_AMP;
        startingPose = new Pose2d(TrajectoryPlans.RedAmpPlan.plan[i][j].waypoint, new Rotation2d(-Math.PI/2.0));  
       } else { 
        destination = VisionConstants.AprilTag.RED_LEFT_SOURCE;
        startingPose = new Pose2d(TrajectoryPlans.RedAmpPlan.plan[i][j].waypoint, new Rotation2d(Math.PI/2.0));
      }
    }
    Utilities.toSmartDashboard("TT Start",startingPose);
    SmartDashboard.putString("TT","init");
    SmartDashboard.putNumber("TT i",i);
    SmartDashboard.putNumber("TT j",j);

    List<Translation2d> waypoints = new ArrayList<>();
    Pose2d aprilTagPose = null;
    Pose2d nearTargetPose = null;
    //Pose2d targetPose = null;
    SmartDashboard.putString("TT","Running");
    //commands = new SequentialCommandGroup();

    if (destination == AprilTag.BLUE_AMP) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.BlueAmpPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_AMP.id());
      nearTargetPose = Utilities.backToPose(aprilTagPose, 1.0);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.RED_AMP) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.RedAmpPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.RED_AMP.id());
      nearTargetPose = Utilities.backToPose(aprilTagPose, 1.0);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.BLUE_RIGHT_SOURCE) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.BlueSourcePlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_RIGHT_SOURCE.id());
      nearTargetPose = Utilities.backToPose(aprilTagPose, 1.0);
    } else if (destination == AprilTag.RED_LEFT_SOURCE) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.RedSourcePlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.RED_LEFT_SOURCE.id());
      nearTargetPose = Utilities.backToPose(aprilTagPose, 1.0);
    } else {
      //targetPose = startingPose; // This will end up doing nothing.
      nearTargetPose = startingPose;
    }
    //if (waypoints.size() > 0)
      SmartDashboard.putString("TT plan", waypoints.toString());
    
    if (waypoints.size() > 0 && !startingPose.equals(nearTargetPose))
      new FollowTrajectoryCommand(robotDrive, poseEstimatorSubsystem, null, startingPose, waypoints, nearTargetPose);
    else 
    {
        RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(new Trajectory());
    }
    if (i < 7) {
      i++;
    } else if (j < 3) {
      j++;
      i=0;
    } else if (destinationID < 1) {
      i = 0;
      j = 0;
      destinationID = 1;
    } else if (allianceID == FieldConstants.BlueAlliance) {
      i = 0;
      j = 0;
      destinationID = 0;
      allianceID = FieldConstants.RedAlliance;
    } else {
      i = 0;
      j = 0;
      destinationID = 0;
      allianceID = FieldConstants.BlueAlliance;
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
    boolean finished = true;
    return finished;
  }
}
