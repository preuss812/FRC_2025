// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



import org.photonvision.PhotonCamera;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;  // To access the Black Box Controller.
import frc.robot.Utilities;

public class GotoAprilTagCommand extends Command {
  
  public class GotoAprilTagConfig {
    private double maxThrottle;
    private double linearP;
    private double linearI;
    private double linearD;
    private double linearF;
    private double linearIZone;
    private double linearTolerance;

    private double maxRotation;
    private double angularP;
    private double angularI;
    private double angularD;
    private double angularF;
    private double angularIZone;
    private double angularTolerance;

    /**
     * default constructor
     */
    public GotoAprilTagConfig() {
      maxThrottle = 0.70;
      linearP = 2.0;
      linearI = 0.0; // linearP/100.0;
      linearD = 0.0; // linearP*10.0;
      linearF = 0.0;
      linearIZone = Units.inchesToMeters(4.0);
      linearTolerance = Units.inchesToMeters(2.0);

      maxRotation = 0.8;
      angularP =  0.35;
      angularI = 0.0; // angularI/100.0;
      angularD = 0.0; //angularP*10.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(10.0);
      angularTolerance = Units.degreesToRadians(5.0);
    }
    public GotoAprilTagConfig setMaxThrottle(double maxThrottle) {this.maxThrottle = maxThrottle; return this; };
    public GotoAprilTagConfig setLinearP(double linearP) {this.linearP = linearP; return this; };
    public GotoAprilTagConfig setLinearI(double linearI) {this.linearI = linearI; return this; };
    public GotoAprilTagConfig setLinearD(double linearD) {this.linearD = linearD; return this; };
    public GotoAprilTagConfig setLinearF(double linearF) {this.linearF = linearF; return this; };
    public GotoAprilTagConfig setLinearIZone(double linearIZone) {this.linearIZone = linearIZone; return this; };
    public GotoAprilTagConfig setLinearTolerance(double linearTolerance) {this.linearTolerance = linearTolerance; return this; };

    public GotoAprilTagConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public GotoAprilTagConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public GotoAprilTagConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public GotoAprilTagConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public GotoAprilTagConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public GotoAprilTagConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public GotoAprilTagConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };

    public double getMaxThrottle() { return maxThrottle; }
    public double getLinearP() { return linearP; }
    public double getLinearI() { return linearI; }
    public double getLinearD() { return linearD; }
    public double getLinearF() { return linearF; }
    public double getLinearIZone() { return linearIZone; }
    public double getLinearTolerance() { return linearTolerance; }
    
    public double getMaxRotation() { return maxRotation; }
    public double getAngularP() { return angularP; }
    public double getAngularI() { return angularI; }
    public double getAngularD() { return angularD; }
    public double getAngularF() { return angularF; }
    public double getAngularIZone() { return angularIZone; }
    public double getAngularTolerance() { return angularTolerance; }
  } // GotoAprilTagConfig Class

  /** Creates a new command to move the robot to the specified pose. */
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DriveSubsystemSRX robotDrive;
  private final PhotonCamera photonCamera;
  private final GotoAprilTagConfig config;
  private final double targetDistance;

  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private Pose2d targetPose;
  private boolean onTarget;
  private boolean debug = false;

  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params photonCamera - the Photon Camera Subsystem used to see the April Tags.
   * @params targetDistance - the distance in meters to be away from the April Tag.
   */
  public GotoAprilTagCommand(PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , double targetDistance) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.photonCamera = photonCamera;
    this.targetDistance = targetDistance;
    onTarget = false;
    this.config = new GotoAprilTagConfig();
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  public GotoAprilTagCommand(PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , double targetDistance
    , GotoAprilTagConfig config) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.photonCamera = photonCamera;
    this.targetDistance = targetDistance;
    onTarget = false;
    this.config = config;
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = true; // Not really but if we dont find a target, this will cause the command to end immediately.
    //debugPID = RobotContainer.m_BlackBox.isSwitchCenter();
    double linearP = config.getLinearP();
    double linearI = config.getLinearI();
    
    var pipelineResult = photonCamera.getLatestResult();
    if (pipelineResult.hasTargets()) {
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Pose2d tagPose = poseEstimatorSubsystem.getAprilTagPose(fiducialId);
      //SmartDashboard.putNumber("TagAmbiguity", target.getPoseAmbiguity());
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0) {
        targetPose = Utilities.backToPose(tagPose, targetDistance);

        xController = new PIDController(
          linearP, // config.getLinearP(),
          linearI, // config.getLinearI(),
          config.getLinearD()
        );
        xController.setIZone(config.getLinearIZone());
        yController = new PIDController(
          linearP, // config.getLinearP(),
          linearI, // config.getLinearI(),
          config.getLinearD()
        );
        yController.setIZone(config.getAngularIZone()); // TODO Needs Tuning.

        rotationController = new PIDController(
          config.getAngularP(),
          config.getAngularI(),
          config.getAngularD()
          );
        rotationController.setTolerance(config.getAngularTolerance()); // did not work, dont understand yet
        rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians).
        onTarget = false;
        if (debug) SmartDashboard.putNumber("G2A dist", targetDistance);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean correctRotation = false; // TODO test with true.
    Translation2d translationErrorToTarget;
    Translation2d translationErrorToTargetCorrectedForRotation;
    double rotationError;
    Rotation2d rotationErrorEstimationToDriveTrain;
    Pose2d estimatedPose;
    Pose2d driveTrainPose;
    double estimatedRotationToDriveTrainRotation;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    estimatedPose = poseEstimatorSubsystem.getCurrentPose();

    // Calculate the X and Y and rotation offsets to the target location
    translationErrorToTarget = new Translation2d( targetPose.getX() - estimatedPose.getX(), targetPose.getY() - estimatedPose.getY());
    
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    rotationError = MathUtil.inputModulus(targetPose.getRotation().getRadians() - estimatedPose.getRotation().getRadians(), -Math.PI, Math.PI);
    if (debug) SmartDashboard.putNumber("G2A R", Units.radiansToDegrees(rotationError));
    if (debug) SmartDashboard.putNumber("G2A X", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("G2A Y", translationErrorToTarget.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationErrorToTarget.getY()) < config.getLinearTolerance()
    &&  Math.abs(rotationError) < config.getAngularTolerance()) {
      // Yes, we have arrived
      if (debug) SmartDashboard.putBoolean("G2A OnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      if (correctRotation) {
        // Rotate the drive X and Y taking into account the difference in the coordinates
        // between the DriveTrain and the PoseEstimator.
        // Calculate the difference in rotation between the PoseEstimator and the DriveTrainPose
        driveTrainPose = robotDrive.getPose();
        estimatedRotationToDriveTrainRotation = estimatedPose.getRotation().getRadians() - driveTrainPose.getRotation().getRadians();
        estimatedRotationToDriveTrainRotation = MathUtil.inputModulus(estimatedRotationToDriveTrainRotation, 0.0, Math.PI*2.0);
        
        if (debug) SmartDashboard.putNumber("G2A P2Derr", estimatedRotationToDriveTrainRotation);

        rotationErrorEstimationToDriveTrain = new Rotation2d(estimatedRotationToDriveTrainRotation);
        translationErrorToTargetCorrectedForRotation = translationErrorToTarget.rotateBy(rotationErrorEstimationToDriveTrain);    // TODO Check sign of rotation.
        xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTargetCorrectedForRotation.getX(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
        ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTargetCorrectedForRotation.getY(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
      } else {
        xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTarget.getX(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
        ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTarget.getY(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
      }
      rotationSpeed = -MathUtil.clamp(-rotationController.calculate(rotationError, 0),-config.getMaxRotation(), config.getMaxRotation()); // TODO Check sign  & Clean up 3 negations :-)
    }
    if (debug) SmartDashboard.putNumber("G2A xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("G2A ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("G2A rSpeed", rotationSpeed);
    robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true); // TODO Verify signs of inputs 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget;
  }
} // GotoAprilTagCommand class
