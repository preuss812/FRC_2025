/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.commands.ShoulderHomeCommand;
import frc.robot.commands.ShoulderRotationCommand;
import frc.robot.commands.SwerveToProcessorCommand;
import frc.robot.commands.VerifyStartingPositionCommand;
import frc.robot.commands.WaitToSeeAprilTagCommand;
import frc.robot.commands.ElbowRotationCommand;
import frc.robot.commands.ExpelAlgaeCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AutoDriveToReefCommand;
import frc.robot.commands.AutonomousStartDelayCommand;
import frc.robot.commands.CompoundArmMovementCommand;
import frc.robot.commands.DriveRobotCommand;
import frc.robot.commands.DriveWithoutVisionCommand;
import frc.robot.commands.ElbowHomeCommand;
import frc.robot.commands.FindAprilTagCommand;
import frc.robot.commands.GotoAprilTagCommand;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.ScoreAlgaeInProcessor;
import frc.robot.commands.GotoProcessorCommand;
//mport frc.robot.commands.RotateRobotAutoCommand;
import frc.robot.commands.PushTowardsWallUltrasonic;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Construct the autonomous command.
 * Keep in mind that this class constructs a command.
 * The code below is called NOT during autonomous but before
 * autonomous begins.  As a result conditionals must be lambda functions
 * or must be based on conditions known at the time of Robot.autonomousInit().
 * The trap is to use getPose() or the like in the middle of the plan which will 
 * get the starting location of the robot, not it's location at the time of the reference.
 */
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  private final DriveSubsystemSRX m_robotDrive;
  private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;
  private final ElbowRotationSubsystem m_ElbowRotationSubsystem;
  private final ShoulderRotationSubsystem m_ShoulderRotationSubsystem;
  private final PingResponseUltrasonicSubsystem m_PingResponseUltrasonicSubsystem;
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
  public static boolean reefCenterSet = false;
  public static double myReefX;
  public static double myReefY;
  public static int m_autoMode = 1; // Default to move 1 meter and stop;

  /**
   * robotHeadingForCameraToReefCenter - helper function for controlling rotation during autonomous driving.
   * @param x - (double) robot x field coordinate.
   * @param y - (double) robot y field coordinate.
   * @return  - (double) the heading in radians from the robot to the reef center.
   */
  public static double robotHeadingForCameraToReefCenter(double x, double y) {
    return MathUtil.angleModulus(
        Math.atan2(myReefY - y,myReefX - x) + VisionConstants.rearCameraHeading);
  }

  public static double robotHeadingForCameraToPose(Pose2d currentPose, Pose2d targetPose) {
    return MathUtil.angleModulus(
      Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()) + VisionConstants.rearCameraHeading
    );
  }

  public static double robotHeadingForCameraToReefCenter(boolean convertToRed, double x, double y) {
    if (convertToRed) {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.redReefCenter.getY() - y, FieldConstants.redReefCenter.getX() - x) + VisionConstants.rearCameraHeading);
    } else {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.blueReefCenter.getY() - y, FieldConstants.blueReefCenter.getX() - x) + VisionConstants.rearCameraHeading);
    }
  }

  public static void setAutoMode() {
    try {
      m_autoMode = Robot.autoChooser.getSelected();
    }
    catch(Exception d) {
      m_autoMode = 1;
    }
    if (!(m_autoMode >= 0 && m_autoMode < TrajectoryPlans.autoNames.size()))
      m_autoMode = 1;
  }

  // Helper function for instant command.
  public static int getAutoMode() {
    return m_autoMode;
  }

  /**
   * set up the reef center for autonomous driving based on the alliance color.
   */
  public static void setReefCenter() {
    if (Utilities.isBlueAlliance()) {
      myReefX = FieldConstants.blueReefCenter.getX();
      myReefY = FieldConstants.blueReefCenter.getY();
      reefCenterSet = true;
    } else if (Utilities.isRedAlliance()) {
      myReefX = FieldConstants.redReefCenter.getX();
      myReefY = FieldConstants.redReefCenter.getY();
      reefCenterSet = true;
    }
  }

  public Autonomous(RobotContainer robotContainer) {
    
    // get the required subsystems for constructing the plans below.
    m_robotDrive = RobotContainer.m_robotDrive;
    m_AlgaeIntakeSubsystem = RobotContainer.m_AlgaeIntakeSubsystem;
    m_ElbowRotationSubsystem = RobotContainer.m_ElbowRotationSubsystem;
    m_ShoulderRotationSubsystem = RobotContainer.m_ShoulderRotationSubsystem;
    m_PoseEstimatorSubsystem = RobotContainer.m_PoseEstimatorSubsystem;
    m_PingResponseUltrasonicSubsystem = RobotContainer.m_PingResponseUltrasonicSubsystem;

    // Set up the alliance first.  Other commands need to know which alliance to operate correctly.
    Utilities.setAlliance();
    setAutoMode();
    setReefCenter();

    // Initialize the robot before moving.
    addCommands(new ParallelCommandGroup(
      new InstantCommand(() -> RobotContainer.setGyroAngleToStartMatch()),
      new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.PRECISION)) // TODO Should be SPEED, not PRECISION
      //new ElbowHomeCommand(m_ElbowRotationSubsystem),
      //new ShoulderHomeCommand(m_ShoulderRotationSubsystem),
      //new InstantCommand(() -> setReefCenter()),
      //new InstantCommand(() -> setAutoMode())
    ));
   
    // For these first 2 modes, just drive 1 meter and wait.
    if (getAutoMode() == TrajectoryPlans.AUTO_MODE_ROBOT_DECIDES || getAutoMode() == TrajectoryPlans.AUTO_MODE_MOVE_OFF_LINE_AND_STOP) {
      addCommands(new DriveWithoutVisionCommand(m_robotDrive, m_PoseEstimatorSubsystem, null, new Pose2d(-1.0, 0, new Rotation2d(0.0))));
      return;
    }

    // For the do-nothing mode, do not move or take any further action.  
    if (getAutoMode() == TrajectoryPlans.AUTO_MODE_DO_NOTHING) {
      return;
    }

    // All other modes involve going to the reef and possibly more.
    addCommands(   
      new CompoundArmMovementCommand(
        m_ElbowRotationSubsystem
        , m_ShoulderRotationSubsystem
        , ElbowConstants.kElbowDrivingWithCoralPosition
        , ShoulderConstants.kShoulderDrivingWithCoralPosition
      ).withTimeout(1.0) // Complete this command after 2 seconds regardless of completion.
    );

    // Perform the initial driving to get from the start line to the reef.
    addCommands(
      new AutoDriveToReefCommand(m_robotDrive, m_PoseEstimatorSubsystem)
    );

    // At the reef now so gently drive into the reef and raise the arm to score the coral
    addCommands(
      new ParallelRaceGroup(
        // TODO: Need to drive gently into the reef.
        new CompoundArmMovementCommand(
          m_ElbowRotationSubsystem, 
          m_ShoulderRotationSubsystem, 
          ElbowConstants.kElbowScoreCoralPosition,
          ShoulderConstants.kShoulderScoreCoralPosition
        ).withTimeout(1.0) // Complete this command after 2 seconds regardless of completion.
      )
    );
    addCommands(new WaitCommand(1.0)); // Wait one second for the coral to roll off the arms.

    addCommands(   
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ArmToDriving")),
      new CompoundArmMovementCommand(
        m_ElbowRotationSubsystem
        , m_ShoulderRotationSubsystem
        , ElbowConstants.kElbowDrivingWithCoralPosition
        , ShoulderConstants.kShoulderDrivingWithCoralPosition
      ).withTimeout(1.0)
    );


    // If the algae is low, try to grab it.
    if (getAutoMode() == TrajectoryPlans.AUTO_MODE_MY_BARGE_TO_FAR_SIDE
    || getAutoMode() == TrajectoryPlans.AUTO_MODE_CENTER_STRAIGHT
    || getAutoMode() == TrajectoryPlans.AUTO_MODE_THEIR_BARGE_TO_FAR_SIDE
    ) {
      // reposition arm to grab low algae and grab the algae.
      addCommands(
        new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ArmToGrabAlgae")),
        new CompoundArmMovementCommand(
          m_ElbowRotationSubsystem, 
          m_ShoulderRotationSubsystem, 
          ElbowConstants.kElbowLowAlgaePosition,
          ShoulderConstants.kShoulderLowAlgaePosition
        ).withTimeout(1.0),
        new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "IntakeAlgae")),
        new AlgaeIntakeCommand(m_AlgaeIntakeSubsystem).withTimeout(AutoConstants.kAlgaeIntakeTime),
        new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ArmToDrive")),
        new CompoundArmMovementCommand(
          m_ElbowRotationSubsystem, 
          m_ShoulderRotationSubsystem, 
          ElbowConstants.kElbowDrivingWithAlgaePosition,
          ShoulderConstants.kShoulderDrivingWithAlgaePosition
        ).withTimeout(2.0) // Complete this command after 2 seconds regardless of completion.
      );

      if (getAutoMode() == TrajectoryPlans.AUTO_MODE_MY_BARGE_TO_FAR_SIDE)
      {
        addCommands(
          new DriveRobotCommand(
            m_robotDrive
            , m_PoseEstimatorSubsystem
            , new Pose2d(-1.0, 0.5, new Rotation2d(0.0))
            , true
            , m_robotDrive.defaultAutoConfig
          )
        );
      }
      // If we are near the processor, drive to it and try to score tha algae
      if (getAutoMode() == TrajectoryPlans.AUTO_MODE_THEIR_BARGE_TO_FAR_SIDE) {
        addCommands(
          new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "swerveToProcessor")),
          new SwerveToProcessorCommand(
            m_robotDrive
            , m_PoseEstimatorSubsystem
          ),
          new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "GotoProcessor")),
          new GotoProcessorCommand(m_robotDrive, m_PoseEstimatorSubsystem, null)// TODO Goto vs SwerveTo

        );

        // Wew are at the processor.  Score the Algae
        addCommands(
          new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ArmToScoreAlgae")),
          new CompoundArmMovementCommand(
            m_ElbowRotationSubsystem, 
            m_ShoulderRotationSubsystem, 
            ElbowConstants.kElbowScoreAlgaeInProcessorPosition,
            ShoulderConstants.kShoulderScoreAlgaeInProcessorPosition
          ).withTimeout(2.0) // Complete this command after 2 seconds regardless of completion.
        );
        addCommands(
          new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ExpelAlgae")),
          new ExpelAlgaeCommand(m_AlgaeIntakeSubsystem).withTimeout(AutoConstants.kAlgaeExpelTime), // Release the algae
          new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "Done"))
        );
      }
    }
  }
}
