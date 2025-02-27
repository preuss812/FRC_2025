// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TrajectoryPlans;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * AutoDriveToReefCommand - return a command for autonomous driving from the start line to the reef
 * based on Field Management alliance (red or blue) and the autonomous plan selected.
 */
public class AutoDriveToReefCommand extends SequentialCommandGroup {
  /** Creates a new AutoDriveToReefCommand. */
  public AutoDriveToReefCommand(
    DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "setStartingPose")),
      TrajectoryPlans.setStartingPoseCommand(poseEstimatorSubsystem),
      new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "PointTowardReef")),
      new PointCameraTowardReefCommand(robotDrive, poseEstimatorSubsystem),
      new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "SwerveToReef")),
      TrajectoryPlans.getReefFacingSwerveCommand(robotDrive, poseEstimatorSubsystem),
      new InstantCommand(() ->robotDrive.drive(0, 0, 0, true, true)),
      new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "GotoReefPose")),
      TrajectoryPlans.gotoFinalPoseCommand(robotDrive, poseEstimatorSubsystem),
      new WaitCommand(2.0)

    );
  }
}
