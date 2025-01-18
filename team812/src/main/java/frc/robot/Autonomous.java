/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.commands.ArmHomeCommand;
import frc.robot.commands.ArmRotationCommand;
import frc.robot.commands.AutonomousStartDelayCommand;
import frc.robot.commands.DriveRobotCommand;
import frc.robot.commands.FindAprilTagCommand;
import frc.robot.commands.ScoreAlgaeInAmp;
import frc.robot.commands.GotoAmpCommand;
//import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.RotateRobotAutoCommand;
//import frc.robot.commands.StopRobotMotion;
//import frc.robot.commands.PushTowardsWall;
import frc.robot.commands.PushTowardsWallUltrasonic;
import frc.robot.Constants.ArmConstants;
//import frc.robot.commands.SwerveToPoseCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
//import frc.robot.Constants.VisionConstants;
//import frc.robot.Constants.VisionConstants.AprilTag;

// Algae:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  private final DriveSubsystemSRX m_robotDrive;
  private final ElbowRotationSubsystem m_ArmRotationSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final PingResponseUltrasonicSubsystem m_PingResponseUltrasonicSubsystem;
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;

  public Autonomous(RobotContainer robotContainer) {

    m_robotDrive = RobotContainer.m_robotDrive;
    m_ArmRotationSubsystem = RobotContainer.m_ArmRotationSubsystem;
    m_ShooterSubsystem = RobotContainer.m_ShooterSubsystem;
    m_PoseEstimatorSubsystem = RobotContainer.m_PoseEstimatorSubsystem;
    m_PingResponseUltrasonicSubsystem = RobotContainer.m_PingResponseUltrasonicSubsystem;

    int autoMode = (int) Math.round(SmartDashboard.getNumber("AutoMode",0));
    if (autoMode < 0 || autoMode >= AutoConstants.mode.length) {
      autoMode = 0; // Out of range values convert back to default mode, 0.
      SmartDashboard.putNumber("AutoMode", autoMode);
    }
    SmartDashboard.putString("AutoModeText", AutoConstants.mode[autoMode]);
  
    if (true || autoMode == 0) {
      /**
       * We are starting with the robots back to the Alliance wall.
       * We hope we can be positioned anywhere along that wall.
       * The command thinks in terms of field coordinates with Blue X= 0 and Red at X=16-ish.
       * Units are in meters and radians.
       * The sequence of steps is:
       * o Set the gyro angle (0 for blue, 180 for red)
       * o Home the arm.
       * o Drive forward 1 meter (+1 meter for Blue, -1 meter for Red).
       * o Turn toward the back of the robot toward the AMP (-90 for Blue, +90 for Red)
       * o Find an april tag (rotate slowly until one is found).  Ideally, seen immediately.
       * o SwerveDrive toward the Amp (if we are close, this is a NO-OP).
       * o GotoPoseDrive to the Amp
       * o Score the Algae.
       * o GotoPoseDrive out of the starting box toward field center.
       */
      final double firstMoveX = 1.84 + 0.50 - DriveConstants.kBackToCenterDistance;
      final double firstMoveY = 0.0;
      final double secondMoveX = 0.0;
      final double secondMoveY = -1.0;
  
      final double finalMoveX = 2.0; // Arbitrary move out of the starting box
      final double finalMoveY = -1.0; // Arbitrary move out of the starting box and away from the wall.
      final double noTagSeenMoveX = 2.0; // Arbitrary move 2 meters forward to exit the starting box.
      final double noTagSeenMoveY = 0.0; // Straight ahead since we do not know where we are.

      Pose2d finalMove;
      Pose2d firstMove;
      Pose2d secondMove;
      Pose2d noTagSeenMove;
      finalMove = new Pose2d(finalMoveX, finalMoveY, new Rotation2d( Math.PI/2.0)); // Pose for robot to face the center of the field.
      firstMove = new Pose2d(firstMoveX, firstMoveY, new Rotation2d(-Math.PI/2.0)); // Pose for robot to be at the april tag.
      secondMove = new Pose2d(secondMoveX, secondMoveY, new Rotation2d(-Math.PI/2.0)); // Pose for robot to be at the april tag.
      noTagSeenMove = new Pose2d(noTagSeenMoveX, noTagSeenMoveY, new Rotation2d( Math.PI/2.0)); // Pose for robot to face the center of the field.

      SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup(
        // Set the gyro starting angle based on alliance and assumed robot placement
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 1)),
        new InstantCommand(() -> RobotContainer.setGyroAngleToStartMatch()),
        new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED)),
        new InstantCommand(() -> Utilities.allianceSetCurrentPose(
          new Pose2d(
            DriveConstants.kBackToCenterDistance,
            DriveConstants.kApproximateStartingY,
            new Rotation2d(DriveConstants.kStartingOrientation)))),

        // Home the arm (should already be homed but this sets the encoder coordinates)
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 2)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmHome")),
        new ArmHomeCommand(RobotContainer.m_ArmRotationSubsystem).withTimeout(3.0),

        // Wait if requested to allow other robots to clear the area.
        //new AutonomousStartDelayCommand(),

        // Drive out based on drivetrain encoders to align with and face the Amp
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 3)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Move1Meter")),
        new DriveRobotCommand(RobotContainer.m_robotDrive, firstMove, false).withTimeout(5.0), // TODO Try controlRotation == true.

        // Rotate toward the Amp.  It's really away from the amp as the camera is on the back of the robot.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 4)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "TurnCameraTowardAmp")),
        new DriveRobotCommand(RobotContainer.m_robotDrive, secondMove, false).withTimeout(5.0), 
        new RotateRobotAutoCommand(RobotContainer.m_robotDrive, -Units.degreesToRadians(70), false).withTimeout(5.0),
        

        // Wait to see apriltag
        //new InstantCommand(() -> Utilities.refineYCoordinate()),  // TODO test this and see if we can reduce or eliminate the wait.
        new WaitCommand(2.8), // TODO reduce or eliminate wait.
        new ConditionalCommand(
          new SequentialCommandGroup(
            
            new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 5)),
            new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "FindAprilTag")),
            new FindAprilTagCommand(
              RobotContainer.m_robotDrive,
              RobotContainer.m_PoseEstimatorSubsystem, 
              AutoConstants.kRotationSpeed).withTimeout(10.0), // This is too slow for just 10 seconds

            // set the robot drive x,y,theta to match the pose estimator (ie use camera to set x,y,theta)
            new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 6)),
            new InstantCommand(() -> robotContainer.alignDriveTrainToPoseEstimator()),

            // Use a trajectory to move close to the amp.
            // This is a place holder for the moment.
            new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 7)),
            new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "SwerveController")),
            //new SwerveToPoseCommand(m_robotDrive, m_PoseEstimatorSubsystem, ampAprilTag),

            // Move to the scoring position
            new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 8)),
            new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "GotoScoringPosition")),
            new ParallelCommandGroup(
              new GotoAmpCommand(m_PoseEstimatorSubsystem, m_robotDrive).withTimeout(3.0),
              
            new ArmRotationCommand(m_ArmRotationSubsystem, ArmConstants.kArmMinPosition)), // TODO raise arm in parallel. 100 fudge factor
            // TODO: Could try raising the arm in parallel with this move to the amp - dph 2024-03-06.

            // Score the Algae.
            // The StopRobotMotion keeps the swerve drive wheels from moving during the scoring.
            new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 9)),
            new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ScoreAlgae")),
            new ParallelDeadlineGroup(
              new ScoreAlgaeInAmp(m_ArmRotationSubsystem, m_ShooterSubsystem),
              new PushTowardsWallUltrasonic(m_robotDrive, m_PingResponseUltrasonicSubsystem)
            ).withTimeout(10.0),

            // Leave the starting box to get more points.
            //new InstantCommand(() -> Utilities.refineYCoordinate()),  // TODO test this and see if we can reduce or eliminate the wait.
            new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 10)),
            new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "LeaveStartBox")),
            new DriveRobotCommand(m_robotDrive, finalMove, true).withTimeout(5.0)
          ),
          // When no tag is in view, just leave the starting box.
          new SequentialCommandGroup(
            // We did not find an april tag so just leave the starting box.
            new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "LeaveStartBox")),
            new DriveRobotCommand(m_robotDrive, noTagSeenMove, true).withTimeout(5.0)
          ),
        m_PoseEstimatorSubsystem.tagInViewSupplier
        ), // ConditionalCommand if we see a tag or not
        // quiesce the drive and finish.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 0)),
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Done"))
        
      );
      addCommands(fullCommandGroup);
    } else if (autoMode == AutoConstants.LeaveMode) {

      final double firstMoveX = 3.0; // Drive out 3 meters  // For in classroom.
      final double firstMoveY = 0.0;

      Pose2d firstMove;

      firstMove = new Pose2d(firstMoveX, firstMoveY, new Rotation2d(0.0)); // face the center of the field.

      SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup(
        // Set the gyro starting angle based on alliance and assumed robot placement
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 1)),
        new InstantCommand(() -> RobotContainer.setGyroAngleToStartMatch()),
        new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED)),
        new InstantCommand(() -> Utilities.allianceSetCurrentPose(
          new Pose2d(
            DriveConstants.kBackToCenterDistance,
            DriveConstants.kApproximateStartingY,
            new Rotation2d(DriveConstants.kStartingOrientation)))),

        // Home the arm (should already be homed but this sets the encoder coordinates)
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 2)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmHome")),
        new ArmHomeCommand(RobotContainer.m_ArmRotationSubsystem).withTimeout(3.0),

        // Wait if requested to allow other robots to clear the area.
        new AutonomousStartDelayCommand(),

        // Drive out based on drivetrain encoders to align with and face the Amp
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 3)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Move1Meter")),
        new DriveRobotCommand(RobotContainer.m_robotDrive, firstMove, true).withTimeout(5.0), // TODO Try controlRotation == true.
         
        // quiesce the drive and finish.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 0)),
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Done"))
      );
      addCommands(fullCommandGroup);

    } else if (autoMode == AutoConstants.USScoreLeaveMode) {
      final double ampZoneDepth = Units.inchesToMeters(13);
      final double startingBoxDepth = Units.inchesToMeters(76.1);
      final double startingRobotCenterY = FieldConstants.yMax - ampZoneDepth - DriveConstants.kBackToCenterDistance;
      final double startingRobotCenterX = FieldConstants.xMin + startingBoxDepth - DriveConstants.kRobotWidth/2.0 - Units.inchesToMeters(2.0)/* tape */;
      final double startingRobotOrientation = Units.degreesToRadians(-90.0);

      final double firstMoveX = 1.84 - startingRobotCenterX;
      final double firstMoveY = FieldConstants.yMax - startingRobotCenterY;
      final double firstRotation = Units.degreesToRadians(-90.0);
      final double finalMoveX = 2.0; // Arbitrary move out of the starting box
      final double finalMoveY = -0.5; // Arbitrary move out of the starting box and away from the wall.
      final double finalRotation = Units.degreesToRadians(0.0);
      Pose2d finalMove;
      Pose2d firstMove;

      finalMove = new Pose2d(finalMoveX, finalMoveY, new Rotation2d(finalRotation)); // Pose for robot to face the center of the field.
      firstMove = new Pose2d(firstMoveX, firstMoveY, new Rotation2d(firstRotation)); // Pose for robot to be at the april tag.
      Utilities.toSmartDashboard("firstMove", firstMove);
      
      SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup(
        // Set the gyro starting angle based on alliance and assumed robot placement
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 1)),
        new InstantCommand(() -> RobotContainer.setGyroAngleToStartMatchAmp()),
        new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.PRECISION)),
        new InstantCommand(() -> Utilities.allianceSetCurrentPose(
          new Pose2d(
            startingRobotCenterX,
            startingRobotCenterY,
            new Rotation2d(startingRobotOrientation)))),

        // Home the arm (should already be homed but this sets the encoder coordinates)
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 2)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmHome")),
        new ArmHomeCommand(RobotContainer.m_ArmRotationSubsystem).withTimeout(3.0),

        // Wait if requested to allow other robots to clear the area.
        //new AutonomousStartDelayCommand(),

        // Start the arm rising to the shooting position.
                new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "raise")),

        new InstantCommand(()->RobotContainer.m_ArmRotationSubsystem.setPosition(ArmConstants.kArmScoringPosition)),

        // Drive out based on drivetrain encoders to align with and face the Amp
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 3)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "MoveToAmp")),
        new DriveRobotCommand(RobotContainer.m_robotDrive, firstMove, false).withTimeout(5.0), // TODO Try controlRotation == true.      

        // set the robot drive x,y,theta to match the pose estimator (ie use estimated position to set x,y,theta)
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 6)),
        new InstantCommand(() -> robotContainer.alignDriveTrainToPoseEstimator()),

        // Score the Algae.
        // The StopRobotMotion keeps the swerve drive wheels from moving during the scoring.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 9)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ScoreAlgae")),
        new ParallelDeadlineGroup(
          new ScoreAlgaeInAmp(m_ArmRotationSubsystem, m_ShooterSubsystem),
          new PushTowardsWallUltrasonic(m_robotDrive, m_PingResponseUltrasonicSubsystem)
        ).withTimeout(10.0),

        // Leave the starting box to get more points.
        new InstantCommand(() -> Utilities.refineYCoordinate()), // Set pose to be exactly in front of the amp touching the amp wall.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 10)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "LeaveStartBox")),
        new DriveRobotCommand(m_robotDrive, finalMove, true).withTimeout(5.0),

        // quiesce the drive and finish.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 0)),
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Done"))
        
      );
      addCommands(fullCommandGroup);
    } else if (autoMode == AutoConstants.DoNothingMode) {
      // Do what we can given we do not know which alliance we are in.
      // Currently it does nothing but stop the drive train which should already be stopped.
      addCommands(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive));
    } else {
      // Do what we can given we do not know which alliance we are in.
      // Currently it does nothing but stop the drive train which should already be stopped.
      addCommands(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive));
    }
  }

}
