// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.utils.DrivingConfig;
import frc.utils.PreussAutoDrive;

/**
 * This comand drives the robot the specified distance
 * If controlRotation is set, it will try to hold the angle in the move pose.
 * If controlRotation is set, it will just manage the X,Y of the move.
 */
public class DriveRobotCommand extends Command {

  private final DriveSubsystemSRX robotDrive;
  private final Pose2d relativeMove;
  private final boolean controlRotation;
  private final DrivingConfig config;
  private final PreussAutoDrive autoDrive;
  private Pose2d startingPose;
  private Pose2d targetPose;
  private boolean onTarget;
  private boolean simulatingRobot = true;
  private boolean debug = true;
  
  /** Creates a new DriveDistanceCommand. */
  public DriveRobotCommand(DriveSubsystemSRX robotDrive, Pose2d relativeMove, boolean controlRotation) {
    this.robotDrive = robotDrive;
    this.relativeMove = relativeMove;
    this.controlRotation = controlRotation;
    this.config = new DrivingConfig(robotDrive.defaultAutoConfig);
    this.autoDrive = new PreussAutoDrive(robotDrive, RobotContainer.m_PoseEstimatorSubsystem, this.config, simulatingRobot);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    autoDrive.reset();

    // get the robot's current pose from the drivetrain
    if (simulatingRobot) {
      startingPose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();
      autoDrive.setCurrentPose(startingPose);
    } else {
      startingPose = robotDrive.getPose();
    }
    // add the relativeMove to the startingPose // There is an alliance component to this.   
    // I'm assuming the caller has handled it in the relativeMove.
    if (Utilities.isBlueAlliance()) {
    targetPose = new Pose2d(
      startingPose.getX() + relativeMove.getX(),
      startingPose.getY() + relativeMove.getY(),
      startingPose.getRotation().rotateBy(relativeMove.getRotation()));
      if (debug) SmartDashboard.putString("DR all", "Blue");

    } else if (Utilities.isRedAlliance()) {
      // This just inverts the X and Y moves as the field this year is rotated about the center of the field.
      targetPose = new Pose2d(
      startingPose.getX() - relativeMove.getX(),
      startingPose.getY() - relativeMove.getY(),
      startingPose.getRotation().rotateBy(relativeMove.getRotation()));
      if (debug) SmartDashboard.putString("DR all", "Red");

    } else {
      targetPose = startingPose; // Do nothing if we dont have an alliance.
      if (debug) SmartDashboard.putString("DR all", "None");

    }
    
    onTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationError;
    double rotationError = 0.0;
    Pose2d currentPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

   
    currentPose = autoDrive.getCurrentPose();

    if (debug) Utilities.toSmartDashboard("Drive Pose", currentPose);
    if (debug) Utilities.toSmartDashboard("Drive target", targetPose);
    // Calculate the X and Y and rotation offsets to the target location
    translationError = new Translation2d( targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    if (controlRotation)
      rotationError = MathUtil.inputModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians(), -Math.PI, Math.PI);
      if (debug) SmartDashboard.putNumber("Drive R Error", Units.radiansToDegrees(rotationError));
      if (debug) SmartDashboard.putNumber("Drive X Error", translationError.getX());
      if (debug) SmartDashboard.putNumber("Drive Y Error", translationError.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationError.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationError.getY()) < config.getLinearTolerance()
    &&  Math.abs(rotationError) < config.getAngularTolerance()) {
      // Yes, we have arrived
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // TODO fine tune PID Controllers and max speeds      
      xSpeed = autoDrive.calculateClampedX(translationError.getX());
      ySpeed = autoDrive.calculateClampedY(translationError.getY());
      if (controlRotation) {
        rotationSpeed = autoDrive.calculateClampedRotation(rotationError); // TODO Check sign
      } else {
        rotationSpeed = 0.0;
      }
      onTarget = false;
    }
    if (debug) SmartDashboard.putBoolean("Drive OnTarget", onTarget);

    if (debug) SmartDashboard.putNumber("Drive xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("Drive ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("Drive rSpeed", rotationSpeed);
    autoDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoDrive.drive(0, 0, 0, true, true); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget;
  }
}
