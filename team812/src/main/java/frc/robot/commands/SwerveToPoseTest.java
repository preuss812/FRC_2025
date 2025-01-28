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
  private static int destinationID = 0; // 0 = Processor
  
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
    Pose2d startingPose = poseEstimatorSubsystem.getCurrentPose();

    if (allianceID == FieldConstants.BlueAlliance) {
      if (destinationID == 0) {
        destination = VisionConstants.AprilTag.BLUE_PROCESSOR;
        startingPose = TrajectoryPlans.fieldSquareToPose(i,j);
      }
    } else {
      if (destinationID == 0) {
        destination = VisionConstants.AprilTag.RED_PROCESSOR;
        startingPose = TrajectoryPlans.fieldSquareToPose(i,j);

       } 
    }
    Utilities.toSmartDashboard("TT Start",startingPose);
    SmartDashboard.putString("TT AL", allianceID == FieldConstants.BlueAlliance ? "Blue" : "Red");
    SmartDashboard.putString("TT","init");
    SmartDashboard.putNumber("TT i",i);
    SmartDashboard.putNumber("TT j",j);
    int ij[] = TrajectoryPlans.poseToFieldSquare(startingPose);
    SmartDashboard.putNumber("TT ip",i);
    SmartDashboard.putNumber("TT jp",j);


    List<Pose2d> waypoints = new ArrayList<>();
    Pose2d aprilTagPose = null;
    Pose2d nearTargetPose = null;
    //Pose2d targetPose = null;
    SmartDashboard.putString("TT","Running");
    //commands = new SequentialCommandGroup();

    if (destination == AprilTag.BLUE_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.BlueProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_PROCESSOR.id());
      nearTargetPose = Utilities.backToPose(aprilTagPose, 1.0);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.RED_PROCESSOR) {
      waypoints = TrajectoryPlans.planTrajectory(TrajectoryPlans.RedProcessorPlan, startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.RED_PROCESSOR.id());
      nearTargetPose = Utilities.backToPose(aprilTagPose, 1.0);
      //targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else {
      //targetPose = startingPose; // This will end up doing nothing.
      nearTargetPose = startingPose;
    }
    //if (waypoints.size() > 0)
      Utilities.toSmartDashboard("TT Plan", waypoints);
    
    if (waypoints.size() > 0 && !startingPose.equals(nearTargetPose))
      new FollowTrajectoryCommand(robotDrive, poseEstimatorSubsystem, null, startingPose, waypoints, nearTargetPose);
    else 
    {
        RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(new Trajectory());
    }
    i++;
    if (i >= TrajectoryPlans.numXSquares) {
      // Move to the next row
      i = 0;
      j++;
      if (j >= TrajectoryPlans.numYSquares) {
        // Switch alliances and start back at the first square.
        i = 0;
        j = 0;
        allianceID = allianceID == FieldConstants.BlueAlliance ? FieldConstants.RedAlliance : FieldConstants.BlueAlliance;
      }
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
