// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraVisionSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.Line;
import frc.utils.DrivingConfig;
import frc.utils.PreussAutoDrive;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;

public class DriveOnAprilTagProjectionCommand extends Command {
  
  /** 
   * Creates a new command to move the robot along the line that projects perpendicularly from an april tag.
   */
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DriveSubsystemSRX robotDrive;
  private final PhotonCamera visionSubsystem;
  private final XboxController xbox;
  private final DrivingConfig config;
  private final PreussAutoDrive autoDrive;
  private final boolean simulation;
  private Pose2d aprilTagPose = null;
  private double cameraToRobotAngle;
  private boolean commandIsActive = false;
  
  private double p; // The constant P from the normal form for the april tag projection line.
  private double xSign; // The sign of x axis motion toward the april tag.
  private double ySign; // The sign of y axis motion toward the april tag. 
  private double projectionM;
  private double projectionB;
  private Line projection;

  private boolean debug = true;
  private static final double[] simulatedJoystick = new double[] {0.1,0.1,0.1};
  private int simulationCycleCount = 0;
  private static final Pose2d simulationStartingPose = new Pose2d(13.5, 0.6, new Rotation2d(Math.PI));
  private static final double needToBeOnLineDistance = 1.0; // (meters)
  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params photonCamera - the Photon Camera Subsystem used to see the April Tags.
   */
  public DriveOnAprilTagProjectionCommand(
      PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , XboxController xbox
    , DrivingConfig config
    , boolean simulation) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.visionSubsystem = photonCamera;
    this.xbox = xbox;
    this.config = config;
    this.simulation = simulation;
    autoDrive = new PreussAutoDrive(robotDrive, poseEstimatorSubsystem, config, simulation);
    cameraToRobotAngle = VisionConstants.XCAMERA_TO_ROBOT.getRotation().getZ();

    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("DP","Active");
    int fiducialId = CameraVisionSubsystem.NO_TAG_FOUND;
    autoDrive.reset();
    aprilTagPose = null;
    commandIsActive = false;
    p = 0.0;
    xSign = 0.0;
    ySign = 0.0;
    
    if (simulation) {
      fiducialId = 6;
      simulationCycleCount = 0;
      autoDrive.setCurrentPose(simulationStartingPose);
    } else {
      fiducialId = poseEstimatorSubsystem.getBestAprilTag(0.2); 
    }

    if (fiducialId != CameraVisionSubsystem.NO_TAG_FOUND) {
          // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(fiducialId);
      commandIsActive = true;
      
      // Compute the projected line from the april tag.
      // This y = mx+b will be used in execute() to save compute cycles
      // I'm using the "normal form" of the line.
      double theta = aprilTagPose.getRotation().getRadians();
      projection = new Line(aprilTagPose.getTranslation(), Math.atan(aprilTagPose.getRotation().getRadians()));
      
      SmartDashboard.putNumber("DP theta", theta);
      SmartDashboard.putNumber("DP cos", Math.cos(theta));

      p = aprilTagPose.getX()* Math.cos(theta) + aprilTagPose.getY() * Math.sin(theta);
      SmartDashboard.putNumber("DP p", p);

      // We also need to determine which direction we are moving toward the line.
      // In other words, which way is towards the target along the line.
      double cos = Math.cos(theta);
      double sin = Math.sin(theta);
      final double epsilon = 0.0001;
      if (Math.abs(cos) < epsilon) cos = 0.0;
      if (Math.abs(sin) < epsilon) sin = 0.0;
      xSign = -Math.signum(cos);
      ySign = -Math.signum(sin);
      if (xSign !=0 ) {
        projectionM = Math.tan(theta);
        // y = mx+b => y - mx = b;
        projectionB = aprilTagPose.getY() - projectionM * aprilTagPose.getX();
      }
      SmartDashboard.putNumber("DP x", xSign);
      SmartDashboard.putNumber("DP y", ySign);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!commandIsActive) return; // If we did not find an april tag, prevent problems with uninitialized variables below

    Translation2d translationErrorToTarget;
    Pose2d robotPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    Translation2d nominalMove = new Translation2d(1,0);
    Translation2d move;
    Translation2d uncorrectedGoal;
    Translation2d correctedGoal;
    double throttle;
    double desiredRotation;
    double rotationError;

    if (aprilTagPose == null)  return;

    throttle = -xbox.getRightY(); // Driver controls +/-X on the projection line
    if (simulation) {
      throttle = simulatedJoystick[simulationCycleCount++];
      if (simulationCycleCount >= simulatedJoystick.length) simulationCycleCount = 0; // Continue on with the last value
    }
    //rotationSpeed = MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband); // Based on xbox right joystick.
    // ? Should rotation be an automatic calculate to keep the robot facing the tag?

    SmartDashboard.putNumber("DA throttle", throttle);
    
    // Calculate rotation required for the camera to face the april tag.
    robotPose = autoDrive.getCurrentPose();
    desiredRotation = MathUtil.angleModulus(Utilities.getHeading(robotPose.getTranslation(), aprilTagPose.getTranslation() )+ cameraToRobotAngle);
    rotationError = MathUtil.angleModulus(desiredRotation - robotPose.getRotation().getRadians());
    rotationSpeed = autoDrive.calculateClampedRotation(rotationError);

    double distanceToAprilTag = Line.distanceBetweenTwoPoints(robotPose.getTranslation(), aprilTagPose.getTranslation());
    double distanceToProjection = projection.distanceFromPointToLine(robotPose.getTranslation());

    
    Transform2d intoAprilTag = new Transform2d(aprilTagPose, new Pose2d(0,0,new Rotation2d(0)));
    Pose2d robotPoseNoRotation = new Pose2d(robotPose.getTranslation(), new Rotation2d(0));
    Pose2d robotPoseApril = robotPoseNoRotation.transformBy(intoAprilTag);
    
    // By default, Split the throttle 50,50 between getting toward the line and moving along the line
    double towardProjectionPortion = Math.sqrt(2.0)/2.0;
    double parallelToProjectionPortion = 1.0 - towardProjectionPortion;
    
    if (robotPoseApril.getX() > robotPoseApril.getY()) {
      // we are getting close so use proportionately more to get on the projectionline
      // Note: this does not account for robot center to robot perimiter.
      towardProjectionPortion = distanceToProjection/(distanceToAprilTag + distanceToProjection);
      parallelToProjectionPortion = 1.0 - towardProjectionPortion;
    }

    Pose2d scaledRobotPoseApril = new Pose2d(robotPoseApril.getX() * towardProjectionPortion* throttle, robotPoseApril.getY() * parallelToProjectionPortion * throttle, robotPoseApril.getRotation());
    translationErrorToTarget = scaledRobotPoseApril.transformBy(intoAprilTag).getTranslation();
  
    
    if (debug) SmartDashboard.putNumber("DA X", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("DA Y", translationErrorToTarget.getY());
    
    xSpeed = autoDrive.calculateClampedX(translationErrorToTarget.getX());
    ySpeed = autoDrive.calculateClampedY(translationErrorToTarget.getY());
  
    if (debug) SmartDashboard.putNumber("DA xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("DA ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("DA rSpeed", rotationSpeed);
    autoDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoDrive.drive(0, 0, 0, true, true);
        SmartDashboard.putString("DP","Done");
    commandIsActive = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (aprilTagPose == null);
  }

  public boolean isActive() {
    return commandIsActive;
  }
}
