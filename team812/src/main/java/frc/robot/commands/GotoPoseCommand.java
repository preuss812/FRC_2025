// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.DrivingConfig;
import frc.utils.PreussAutoDrive;
import frc.robot.RobotContainer;
import frc.robot.Utilities;

public class GotoPoseCommand extends Command {
  
  /** Creates a new command to move the robot to the specified pose. */
  protected final DriveSubsystemSRX robotDrive;
  protected final PoseEstimatorSubsystem poseEstimatorSubsystem;
  protected Pose2d targetPose;
  private final DrivingConfig config;
  private final PreussAutoDrive autoDrive;
  ;
  private boolean onTarget;
  private boolean controlRotation = true; // This is a holdover from when controlRotation was not working well.
  private boolean debug = true; // turn on/off SmartDashBoard feedback
  private boolean simulatingRobot = RobotContainer.isSimulation; // force robot starting position and april tag number for debugging purposes.
  
  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params targetPose - the target location and orientation for the robot.
   */
  public GotoPoseCommand(
      DriveSubsystemSRX robotDrive
    , PoseEstimatorSubsystem poseEstimatorSubsystem
    , Pose2d targetPose
    , DrivingConfig config) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.targetPose = targetPose;
    this.config = config == null ? robotDrive.defaultAutoConfig : config;

    this.autoDrive = new PreussAutoDrive(robotDrive, poseEstimatorSubsystem, this.config, simulatingRobot);
    onTarget = false;

    addRequirements(robotDrive, poseEstimatorSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = true; // Not really but if we dont find a target, this will cause the command to end immediately.
    
    // Reset the pid controllers
    autoDrive.reset();

    onTarget = false; // Defer this calculation to this.isFinished()
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (onTarget) return; // Prevent code from running if we are there or if there was no apriltag.

    Translation2d translationErrorToTarget;
    double rotationError;
    Pose2d robotPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    // get the robot's position on the field.
    robotPose = autoDrive.getCurrentPose();

    // Calculate the X and Y offsets to the target location
    translationErrorToTarget = new Translation2d( targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());

    // Control rotation to always face directly at the target as it approaches.
    // This is to ensure that we get the most updates to our pose estimator for best possitioning accuracy.
    // Calculate the tangent of the translation error.
    double desiredRotation;
    double errorVectorMagnitude = Math.pow(Math.pow(translationErrorToTarget.getX(),2) + Math.pow(translationErrorToTarget.getY(),2),0.5);
    desiredRotation = MathUtil.angleModulus(Utilities.getHeading(robotPose.getTranslation(), targetPose.getTranslation()));

    // If we are very close to the target, use the target pose.
    // This is an attempt to overcome the problem of low rotation throttle from the pid when we are very close to the target.
    if (errorVectorMagnitude  < 0.5 /* meters */) {
      desiredRotation = targetPose.getRotation().getRadians();
    }
    rotationError = MathUtil.angleModulus(desiredRotation - robotPose.getRotation().getRadians());

    // Make sure the rotation error is between -PI and PI
    if (debug) SmartDashboard.putNumber("G2A R", Units.radiansToDegrees(rotationError));
    if (debug) SmartDashboard.putNumber("G2A X", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("G2A Y", translationErrorToTarget.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationErrorToTarget.getY()) < config.getLinearTolerance()
    &&  ((!controlRotation) || (Math.abs(rotationError) < config.getAngularTolerance()))) {
      // We are close enough.  Stop the robot and the command.
      if (debug) SmartDashboard.putBoolean("G2A OnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // We are not close enough yet./

      // Transform the error into targetPose coordinates
      Translation2d targetPoseErrorVector = translationErrorToTarget.rotateBy(targetPose.getRotation()); //  vector from apriltag to the robot rotated in april tag space

      // Calculate the speeds in the coordinates system defined by the april tag
      // It is important that clipping occurs here and not below as clipping in the field coordinates 
      // will lead to paths that may initially veer away from the target.
      double xSpeedTargetPose = autoDrive.calculateX(targetPoseErrorVector.getX());
      double ySpeedTargetPose = autoDrive.calculateY(targetPoseErrorVector.getY());

      SmartDashboard.putNumber("GAAX", xSpeedTargetPose);
      SmartDashboard.putNumber("GAAY", ySpeedTargetPose);
      
      // Enforce maThrottle by scaling the magnitude of the error vector.
      // If we clamped instead we would see a slightly off initial angle when clipping happens.
      if (Math.sqrt(xSpeedTargetPose*xSpeedTargetPose + ySpeedTargetPose*ySpeedTargetPose) > config.getMaxThrottle()) {
        double scaleFactor =  config.getMaxThrottle() / Math.sqrt(xSpeedTargetPose*xSpeedTargetPose + ySpeedTargetPose*ySpeedTargetPose);
        xSpeedTargetPose *= scaleFactor;
        ySpeedTargetPose *= scaleFactor;
      }

      // Rotate the calculated speeds back to field coordinates.
      Translation2d targetPoseSpeeds = new Translation2d(xSpeedTargetPose,ySpeedTargetPose);
      Translation2d unrotatedSpeedsPose = targetPoseSpeeds.rotateBy(targetPose.getRotation().times(-1));
      
      // Use the unrotated speeds to control the robot.  It is possible that these speeds could exceed te max throttle but dont
      // clip them unless absolutely necessary to avoid artifacts in the paths.
      // The extra magnitude is more or less limited to sqrt(2)*maxThrottle
      xSpeed = MathUtil.clamp(unrotatedSpeedsPose.getX(), -1.0, 1.0);
      ySpeed = MathUtil.clamp(unrotatedSpeedsPose.getY(), -1.0, 1.0);

      // We are controlling rotation whether we use it for "onTarget" calculations or not.
      rotationSpeed = autoDrive.calculateClampedRotation(rotationError);
    }
    if (debug) SmartDashboard.putNumber("G2A xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("G2A ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("G2A rSpeed", rotationSpeed);
    autoDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget;
  }
} // GotoPoseCommand class
