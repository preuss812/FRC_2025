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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryPlans;
import frc.robot.Utilities;

public class SwerveToPoseCommand2 extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final AprilTag destination;
  private SequentialCommandGroup commands;
  private final double finalDistanceToAmp    = 1.0; // Meters
  private final double finalDistanceToSource = 1.0; // Meters
  private enum Action{FIRST_TURN, travel, lastTurn}

  /** Creates a new SwerveToPoseCommand2. */
  public SwerveToPoseCommand2(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    AprilTag destination
  ) {
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.destination = destination;

    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(robotDrive, poseEstimatorSubsystem);  // This may be a problem due to self cancellation.
    // no requirements added because of self-cancellation problems when running sub-commands.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Action action = Action.FIRST_TURN;
    Pose2d startingPose = poseEstimatorSubsystem.getCurrentPose();
    
    List<Translation2d> waypoints = new ArrayList<>();
    Pose2d aprilTagPose = null;
    Pose2d targetPose = null;
    SmartDashboard.putString("SW","init");
    Utilities.toSmartDashboard("SW Start",startingPose);
    commands = new SequentialCommandGroup();

    if (destination == AprilTag.BLUE_PROCESSOR) {
      //waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.BlueProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_PROCESSOR.id());
      targetPose = Utilities.backToPose(aprilTagPose, finalDistanceToAmp);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.RED_PROCESSOR) {
      //aypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.RedProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.RED_PROCESSOR.id());
      targetPose = Utilities.backToPose(aprilTagPose, finalDistanceToAmp);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else {
      //targetPose = startingPose; // This will end up doing nothing.
      targetPose = startingPose;
    }
    Pose2d rotatedStartingPose = startingPose;
    Pose2d rotatedTargetPose = targetPose;
    if (waypoints.size() > 1) {
      Rotation2d faceFirstWaypoint = new Rotation2d(
        waypoints.get(0).getX() - startingPose.getX(),
        waypoints.get(0).getY() - startingPose.getY()
      );
      rotatedStartingPose = new Pose2d(startingPose.getTranslation(), faceFirstWaypoint);
      //commands.addCommands(new RotateRobotCommand(robotDrive, faceFirstWaypoint.getRadians(), false));
    }
    if (waypoints.size() > 1) {
      Rotation2d faceTargetPose = new Rotation2d(
        targetPose.getX() - waypoints.get(waypoints.size() - 1).getX(),
        targetPose.getY() - waypoints.get(waypoints.size() - 1).getY()
      );
      rotatedTargetPose = new Pose2d(targetPose.getTranslation(), faceTargetPose);
      //commands.addCommands(new RotateRobotCommand(robotDrive, faceFirstWaypoint.getRadians(), false));
    }
    if (waypoints.size() > 0 && !startingPose.equals(targetPose)) {
      //new FollowTrajectoryCommand(robotDrive, poseEstimatorSubsystem, null, rotatedStartingPose, waypoints, rotatedTargetPose);
    } else {
        RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(new Trajectory());
    }
    if (startingPose != null && waypoints.size() > 0 && targetPose != null) {
      //commands.addCommands(new FollowTrajectoryCommand(robotDrive, poseEstimatorSubsystem, null, startingPose, waypoints, targetPose));
    }
    // This effectively forks off the command from our thread.
    // This command will stay active until the FollowTrajectoryCommand finishes 
    // returning isFinished() only when the FTC command returns isFinished==true;
    commands.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // This try/catch handles the case where the FollowTrajectoryCommand no longer exists.
    try {
      commands.cancel();
    } catch (Exception e) {
    }
    SmartDashboard.putBoolean("SW Interrupted", interrupted);
    SmartDashboard.putString("SW","Done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = true;
    try {
      finished = commands.isFinished();
    } catch (Exception e) {
      finished = true;
    }
    return finished;
  }
}
