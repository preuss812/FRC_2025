package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;
import frc.utils.PreussAutoDrive;

/**
 * This command drives the robot in a circular path.
 */
public class DriveCircleThrottle extends Command {

  private Integer count;
  private Pose2d circleCenter;
  private Pose2d startingPose;
  private final static Pose2d dummy = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private static Integer initcalls = 0;
  private static Integer executecalls = 0;
  private static Integer isFinishedcalls = 0;

  // Define the radius of the circle (in meters)
  private  double radius;
  private final DriveSubsystemSRX robotDrive;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DrivingConfig config;

  private final double speed;
  private final PreussAutoDrive autoDrive;
  private boolean debug = true;

  /** Creates a new DriveCircleCommand. */
  public DriveCircleThrottle(DriveSubsystemSRX robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem, DrivingConfig  config, double speed) {
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.config = config;
    this.speed = speed;
    autoDrive = new PreussAutoDrive(robotDrive, poseEstimatorSubsystem, config);

    SmartDashboard.putNumber("inits", initcalls);
    SmartDashboard.putNumber("executes", executecalls);
    SmartDashboard.putNumber("isFinished", isFinishedcalls);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the current pose of the robot as the center of the circle.
    double circumference = 360.0/50.0 * speed;
    radius = circumference / (2 * Math.PI);

    startingPose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();

    // Optional: Offset the center to define where the circle will start (if you want).
    // For example, moving the center by 1 meter on the X-axis:
    circleCenter = new Pose2d(startingPose.getX() - radius, startingPose.getY(), startingPose.getRotation());
    // Not using the circle center yet but will probably need it to control error.

    count = 0;
    initcalls++;
    executecalls = 0;
    SmartDashboard.putNumber("inits", initcalls);

    // No need to call super.initialize() because the GotoPoseCommand constructor handles that
  }

  @Override
  public void execute() {
    double throttleMagnitude = speed / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    // Calculate the joystick positions to follow the circular path
    double angle = Units.degreesToRadians(count >= 720 ? 0.0 : count%360);  // Convert to radians
    double dx = Math.cos(angle+0.5) - Math.cos(angle-0.5);
    double dy = Math.sin(angle+0.5) - Math.sin(angle-0.5);
    double xSpeed = dx * throttleMagnitude;
    double ySpeed = dy * throttleMagnitude;


    // Make sure the rotation error is between -PI and PI
    // get the robot's position on the field.
    Pose2d robotPose = autoDrive.getCurrentPose();
    double targetX = circleCenter.getX() + radius * Math.cos(angle);
    double targetY = circleCenter.getY() + radius * Math.sin(angle);

    // Calculate the X and Y offsets to the target location
    //translationErrorToTarget = new Translation2d( targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());
    Translation2d   translationErrorToTarget = new Translation2d( robotPose.getX() - targetX, robotPose.getY() - targetY);
    if (debug) SmartDashboard.putNumber("AutoDrive R", Units.radiansToDegrees(0));
    if (debug) SmartDashboard.putNumber("AutoDrive X", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("AutoDrive Y", translationErrorToTarget.getY());
    autoDrive.drive(xSpeed, ySpeed, 0.0, true, false);

    count++;  // Increment the angle

    executecalls++;
    SmartDashboard.putNumber("executes", executecalls);
  }
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    autoDrive.drive(0.0, 0.0, 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    isFinishedcalls++;
    Pose2d pose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();
    // Finish when the angle has completed a full circle (360 degrees)
    return count >= 720;
    
  }
}
