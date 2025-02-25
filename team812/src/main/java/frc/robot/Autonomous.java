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
        Math.atan2(myReefY - y,myReefX - x) + VisionConstants.cameraHeading);
  }

  public static double robotHeadingForCameraToReefCenter(boolean convertToRed, double x, double y) {
    if (convertToRed) {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.redReefCenter.getY() - y, FieldConstants.redReefCenter.getX() - x) + VisionConstants.cameraHeading);
    } else {
      return MathUtil.angleModulus(
        Math.atan2(FieldConstants.blueReefCenter.getY() - y, FieldConstants.blueReefCenter.getX() - x) + VisionConstants.cameraHeading);
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
   
    addCommands(   
      new CompoundArmMovementCommand(
        m_ElbowRotationSubsystem
        , m_ShoulderRotationSubsystem
        , ElbowConstants.kElbowDrivingWithCoralPosition
        , ShoulderConstants.kShoulderDrivingWithCoralPosition
      ).withTimeout(2.0) // Complete this command after 2 seconds regardless of completion.
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
        ).withTimeout(2.0) // Complete this command after 2 seconds regardless of completion.
      )
    );
    addCommands(new WaitCommand(1.0)); // Wait one second for the coral to roll off the arms.

    addCommands(   
      new CompoundArmMovementCommand(
        m_ElbowRotationSubsystem
        , m_ShoulderRotationSubsystem
        , ElbowConstants.kElbowDrivingWithCoralPosition
        , ShoulderConstants.kShoulderDrivingWithCoralPosition
      )
    );

    // reposition arm to grab low algae and grab the algae.
    addCommands(
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ArmToGrabAlgae")),
      new CompoundArmMovementCommand(
        m_ElbowRotationSubsystem, 
        m_ShoulderRotationSubsystem, 
        ElbowConstants.kElbowLowAlgaePosition,
        ShoulderConstants.kShoulderLowAlgaePosition
      ),
      new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "IntakeAlgae")),
      new AlgaeIntakeCommand(m_AlgaeIntakeSubsystem).withTimeout(AutoConstants.kAlgaeIntakeTime),
      new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ArmToDrive")),
      new CompoundArmMovementCommand(
        m_ElbowRotationSubsystem, 
        m_ShoulderRotationSubsystem, 
        ElbowConstants.kElbowDrivingWithAlgaePosition,
        ShoulderConstants.kShoulderDrivingWithAlgaePosition
      ).withTimeout(2.0) // Complete this command after 2 seconds regardless of completion.
    );

   /*
    addCommands(
      new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "swerveToProcessor")),
      new SwerveToProcessorCommand(
        m_robotDrive
        , m_PoseEstimatorSubsystem
      ),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "GotoProcessor")),
      new GotoProcessorCommand(m_robotDrive, m_PoseEstimatorSubsystem, null), // TODO Goto vs SwerveTo
      new WaitCommand(2.0)

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
    addCommands()
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "ExpelAlgae")),
      new ExpelAlgaeCommand(m_AlgaeIntakeSubsystem).withTimeout(AutoConstants.kAlgaeExpelTime), // Release the algae
      new WaitCommand(2.0),
      new InstantCommand(() ->SmartDashboard.putString("AutoCommand", "Done"))
    );
    */
  }
}
