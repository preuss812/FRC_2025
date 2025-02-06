/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.photonvision.PhotonCamera;

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
import frc.robot.commands.ShoulderHomeCommand;
import frc.robot.commands.VerifyStartingPositionCommand;
import frc.robot.commands.WaitToSeeAprilTagCommand;
import frc.robot.commands.ElbowRotationCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AutonomousStartDelayCommand;
import frc.robot.commands.CompoundArmMovementCommand;
import frc.robot.commands.DriveRobotCommand;
import frc.robot.commands.ElbowHomeCommand;
import frc.robot.commands.FindAprilTagCommand;
import frc.robot.commands.GotoAprilTagCommand;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.ScoreAlgaeInProcessor;
import frc.robot.commands.GotoProcessorCommand;
import frc.robot.commands.RotateRobotAutoCommand;
import frc.robot.commands.PushTowardsWallUltrasonic;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShoulderConstants;

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
  private final PhotonCamera m_PhotonCamera;

  public Autonomous(RobotContainer robotContainer) {
    
    // get the required subsystems for constructing the plans below.
    m_robotDrive = RobotContainer.m_robotDrive;
    m_AlgaeIntakeSubsystem = RobotContainer.m_AlgaeIntakeSubsystem;
    m_ElbowRotationSubsystem = RobotContainer.m_ElbowRotationSubsystem;
    m_ShoulderRotationSubsystem = RobotContainer.m_ShoulderRotationSubsystem;
    m_PoseEstimatorSubsystem = RobotContainer.m_PoseEstimatorSubsystem;
    m_PingResponseUltrasonicSubsystem = RobotContainer.m_PingResponseUltrasonicSubsystem;
    m_PhotonCamera = RobotContainer.m_camera;

    int autoMode = Robot.autoChooser.getSelected();
    if (autoMode < 0 || autoMode >= TrajectoryPlans.autoNames.size()) {
      autoMode = 0; // Out of range values convert back to default mode, 0.
    } 
    SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup();

    // Add the basic robot initialization.
    this.addCommands(
      new ParallelCommandGroup(
      new InstantCommand(() -> RobotContainer.setGyroAngleToStartMatch()),
      new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.PRECISION)), // TODO Should be SPEED, not PRECISION
      new InstantCommand(() -> Utilities.allianceSetCurrentPose(
        new Pose2d(
          FieldConstants.blueStartLine, // This will be transformed to red if we are in the red alliance.
          FieldConstants.yCenter,       // This will be transformed to red if we are in the red alliance
          new Rotation2d(DriveConstants.kStartingOrientation)))),

        // Home the arm (should already be homed but this sets the encoder coordinates)
        new ElbowHomeCommand(RobotContainer.m_ElbowRotationSubsystem).withTimeout(ElbowConstants.kElbowHomeTimeout),
        new ShoulderHomeCommand(RobotContainer.m_ShoulderRotationSubsystem).withTimeout(ShoulderConstants.kShoulderHomeTimeout)
      )
    );
    this.addCommands(   
      new CompoundArmMovementCommand(
        m_ElbowRotationSubsystem
        , m_ShoulderRotationSubsystem
        , ElbowConstants.kElbowDrivingWithCoralPosition
        , ShoulderConstants.kShoulderDrivingWithCoralPosition
        , false
        )
      );

    if (autoMode == AutoConstants.kDoNothing) {
      // We will do nothing!!!.  Hopefully this does not get used.
    } else if (autoMode == AutoConstants.kDriveOffTheLineAndStop) {
      // Drive 1 meter and stop.
      this.addCommands( new DriveRobotCommand(m_robotDrive, new Pose2d(1.0, 0, new Rotation2d(0)), true));
    } else if (autoMode == AutoConstants.kRobotMakesThePlan) {
      // Look for an april tag and if we see one, use the corresponding path from the 6 predefined paths.
      // but for now, it's doing nothing TODO add something here.
    } else { 
      // One of the 6 predefined paths to 
      
      // First wait until we see an april tag.  This should return right away because if things are set up
      // correctly, we will always have an april tag in view.
      // This is here in case the placement on the field and the plan selected do not match.
      this.addCommands(new WaitToSeeAprilTagCommand(m_PoseEstimatorSubsystem));
      
      // Check that we see the expected april tag for the plan we are trying to execute
      // This command will wait for 15 seconds if we dont see the expected tag
      // which will cause no more action during autonomous.
      addCommands(new VerifyStartingPositionCommand(m_PoseEstimatorSubsystem, autoMode)) ;
      

      // Drive to near the selected april tag.
      addCommands(TrajectoryPlans.autoPlans.get(autoMode));

      // Now that we are near the tag, drive right up to the reef using the tag on the reef to precisely align
      // the robot to the algae.  Note that GotoAprilTagCommand goes to whatever tag it sees.
      // Also note that if we are too close to the target, the camera may not see it.
      addCommands(new GotoAprilTagCommand(
          m_PoseEstimatorSubsystem,
          m_robotDrive,
          m_PhotonCamera,
          DriveConstants.kFrontToCenterDistance,
          m_robotDrive.debugAutoConfig, // TODO change to defaultAutoConfig for faster driving.
          false // Not simulation
        )
      );

      addCommands(
        new CompoundArmMovementCommand(
          m_ElbowRotationSubsystem, 
          m_ShoulderRotationSubsystem, 
          ElbowConstants.kElbowScoreCoralPosition,
          ShoulderConstants.kShoulderScoreCoralPosition,
          false // This is not simulation
        )
      );
      addCommands(new WaitCommand(1.0)); // Wait one second for the coral to roll off the arms.

      // grab the algae
      addCommands(
        new CompoundArmMovementCommand(
          m_ElbowRotationSubsystem, 
          m_ShoulderRotationSubsystem, 
          ElbowConstants.kElbowLowAlgaePosition,
          ShoulderConstants.kShoulderLowAlgaePosition,
          false // This is not simulation
        ),
        new AlgaeIntakeCommand(m_AlgaeIntakeSubsystem).withTimeout(AutoConstants.kAlgaeIntakeTime),
        new CompoundArmMovementCommand(
          m_ElbowRotationSubsystem, 
          m_ShoulderRotationSubsystem, 
          ElbowConstants.kElbowLowAlgaePosition,
          ShoulderConstants.kShoulderLowAlgaePosition,
          false // This is not simulation
        ),
        new GotoProcessorCommand(m_PoseEstimatorSubsystem, m_robotDrive), // This could be dangerous 
        new CompoundArmMovementCommand(
          m_ElbowRotationSubsystem, 
          m_ShoulderRotationSubsystem, 
          ElbowConstants.kElbowLowAlgaePosition,
          ShoulderConstants.kShoulderLowAlgaePosition,
          false // This is not simulation
        ),
        new GotoPoseCommand(
          m_PoseEstimatorSubsystem,
          m_robotDrive,
          new Pose2d(0,0,new Rotation2d(0)) // Need a process pose with the right orientation and distance from the processor
        )
      );

    }
    addCommands(fullCommandGroup);


  }

}
